/*
 * mic_test.ino
 *
 * Second proof of concept for a freETarget minimalistic port to the ESP32:
 * four KY-037 microphone boards on the MCPWM capture inputs. For every bang
 * it prints the raw arrival time differences, so the behaviour of the
 * microphones can be judged before the position calculation is ported.
 *
 * Wiring (KY-037 powered from 3V3, NOT 5V; A0 stays unconnected):
 *
 *   KY-037 +   -> 3V3
 *   KY-037 G   -> GND
 *   KY-037 D0  -> North: GPIO 4, East: GPIO 5, South: GPIO 6, West: GPIO 7
 *
 * North, East and South use the three capture channels of MCPWM group 0.
 * West uses group 1, which has its own free running timer. North is routed
 * to a second channel in group 1 as well, so West is measured against North
 * on the same timer and the offset between the timers drops out.
 *
 * Board: ESP32-S3 Dev Module (ESP32-S3-WROOM-1), arduino-esp32 core 3.x
 */
#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"

#define PIN_NORTH     GPIO_NUM_4
#define PIN_EAST      GPIO_NUM_5
#define PIN_SOUTH     GPIO_NUM_6
#define PIN_WEST      GPIO_NUM_7

#define N_GROUPS      2
#define N_CHANNELS    5
#define N_SENSORS     4
#define CH_NORTH      0
#define CH_NORTH_REF  4                 // North again, on the timer of group 1

#define WINDOW_MS     20                // Collect edges this long after the first one
#define QUIET_MS      300               // Arm again after this long without a single edge
#define SPEED_MM_S    343000.0          // Speed of sound at 20 C, for the mm column only

typedef struct
{
  const char *name;
  gpio_num_t  pin;
  int         group;
} channel_t;

static const channel_t channel[N_CHANNELS] =
{
  { "N", PIN_NORTH, 0 },
  { "E", PIN_EAST,  0 },
  { "S", PIN_SOUTH, 0 },
  { "W", PIN_WEST,  1 },
  { "N", PIN_NORTH, 1 },
};

static mcpwm_cap_timer_handle_t   cap_timer[N_GROUPS];
static mcpwm_cap_channel_handle_t cap_chan[N_CHANNELS];
static volatile uint32_t cap_value[N_CHANNELS];   // Timer value of the first edge
static volatile uint32_t cap_rising[N_CHANNELS];  // Polarity of the first edge
static volatile uint32_t cap_edges[N_CHANNELS];   // Number of edges seen, shows ringing
static volatile uint32_t cap_mask;                // Channels that have their first edge
static int      idle_level[N_SENSORS];            // Level of D0 in silence, read at startup
static uint32_t resolution_hz;
static uint32_t shot_number;

static bool IRAM_ATTR on_capture(mcpwm_cap_channel_handle_t chan, const mcpwm_capture_event_data_t *edata, void *user)
{
  uint32_t ch = (uint32_t)(uintptr_t)user;

  cap_edges[ch] = cap_edges[ch] + 1;
  if ( (cap_mask & (1u << ch)) == 0 )   // Keep the first edge only, as the Mega ISRs do
  {
    cap_value[ch]  = edata->cap_value;
    cap_rising[ch] = (edata->cap_edge == MCPWM_CAP_EDGE_POS);
    cap_mask = cap_mask | (1u << ch);
  }
  return false;
}

static bool check(const char *what, esp_err_t err)
{
  if ( err != ESP_OK )
  {
    Serial.printf("FAILED %s: %s\r\n", what, esp_err_to_name(err));
  }
  return err == ESP_OK;
}

static bool init_capture(void)
{
  for (int g = 0; g != N_GROUPS; g++)
  {
    mcpwm_capture_timer_config_t timer_config = {};
    timer_config.group_id = g;
    timer_config.clk_src  = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
    if ( !check("new_capture_timer", mcpwm_new_capture_timer(&timer_config, &cap_timer[g])) )
    {
      return false;
    }
  }

  for (int ch = 0; ch != N_CHANNELS; ch++)
  {
    mcpwm_capture_channel_config_t chan_config = {};
    chan_config.gpio_num       = channel[ch].pin;
    chan_config.prescale       = 1;
    chan_config.flags.pos_edge = true;
    chan_config.flags.neg_edge = true;
    if ( !check("new_capture_channel", mcpwm_new_capture_channel(cap_timer[channel[ch].group], &chan_config, &cap_chan[ch])) )
    {
      return false;
    }

    mcpwm_capture_event_callbacks_t callbacks = {};
    callbacks.on_cap = on_capture;
    if ( !check("register_callbacks", mcpwm_capture_channel_register_event_callbacks(cap_chan[ch], &callbacks, (void *)(uintptr_t)ch))
      || !check("channel_enable", mcpwm_capture_channel_enable(cap_chan[ch])) )
    {
      return false;
    }
  }

  for (int g = 0; g != N_GROUPS; g++)
  {
    if ( !check("timer_enable", mcpwm_capture_timer_enable(cap_timer[g]))
      || !check("timer_start", mcpwm_capture_timer_start(cap_timer[g])) )
    {
      return false;
    }
  }

  return check("get_resolution", mcpwm_capture_timer_get_resolution(cap_timer[0], &resolution_hz));
}

static void arm(void)
{
  for (int ch = 0; ch != N_CHANNELS; ch++)
  {
    cap_edges[ch] = 0;
  }
  cap_mask = 0;                         // Single write, arms all channels at once
}

/*
 * Wait until the echoes are gone: no edge on any channel and every output
 * back at its idle level for QUIET_MS in a row. Arming earlier starts a
 * measurement in the middle of a sound, which gives meaningless times.
 */
static void wait_for_silence(void)
{
  uint32_t quiet_since = millis();
  uint32_t last_edges  = 0;

  while ( (millis() - quiet_since) < QUIET_MS )
  {
    uint32_t edges = 0;
    bool     idle  = true;

    for (int ch = 0; ch != N_SENSORS; ch++)
    {
      edges = edges + cap_edges[ch];
      idle  = idle && (gpio_get_level(channel[ch].pin) == idle_level[ch]);
    }
    if ( (edges != last_edges) || !idle )
    {
      last_edges  = edges;
      quiet_since = millis();
    }
    delay(1);
  }
}

/*
 * Arrival time of a sensor relative to North, in timer ticks. West is
 * compared to the North channel that shares its timer.
 */
static bool ticks_after_north(int ch, uint32_t mask, int32_t *ticks)
{
  int north = (channel[ch].group == 0) ? CH_NORTH : CH_NORTH_REF;

  if ( ((mask & (1u << ch)) == 0) || ((mask & (1u << north)) == 0) )
  {
    return false;
  }
  *ticks = (int32_t)(cap_value[ch] - cap_value[north]);
  return true;
}

static void report(void)
{
  uint32_t mask = cap_mask;
  double   us_per_tick = 1e6 / (double)resolution_hz;
  double   mm_per_tick = SPEED_MM_S / (double)resolution_hz;

  shot_number++;
  Serial.printf("\r\nshot %lu\r\n", (unsigned long)shot_number);
  for (int ch = 0; ch != N_SENSORS; ch++)
  {
    int32_t ticks;

    if ( (mask & (1u << ch)) == 0 )
    {
      Serial.printf("  %s  no edge\r\n", channel[ch].name);
    }
    else if ( ticks_after_north(ch, mask, &ticks) )
    {
      Serial.printf("  %s  %+8ld ticks  %+9.2f us  %+8.2f mm   first edge %s, %lu edges\r\n",
                    channel[ch].name, (long)ticks, ticks * us_per_tick, ticks * mm_per_tick,
                    cap_rising[ch] ? "rising" : "falling", (unsigned long)cap_edges[ch]);
    }
    else
    {
      Serial.printf("  %s  no North to compare with   first edge %s, %lu edges\r\n",
                    channel[ch].name, cap_rising[ch] ? "rising" : "falling", (unsigned long)cap_edges[ch]);
    }
  }

  if ( (mask & (1u << CH_NORTH)) && (mask & (1u << CH_NORTH_REF)) )
  {
    Serial.printf("  timer offset group 1 - group 0: %ld ticks\r\n",
                  (long)(int32_t)(cap_value[CH_NORTH_REF] - cap_value[CH_NORTH]));
  }
}

void setup(void)
{
  Serial.begin(115200);
  delay(1000);
  Serial.printf("\r\nfreETarget ESP32 microphone test\r\n");

  if ( !init_capture() )
  {
    Serial.printf("Capture setup failed, stopping\r\n");
    while ( true )
    {
      delay(1000);
    }
  }
  Serial.printf("Capture resolution: %lu Hz = %.2f ns per tick\r\n",
                (unsigned long)resolution_hz, 1e9 / (double)resolution_hz);

  Serial.printf("Idle levels:");
  for (int ch = 0; ch != N_SENSORS; ch++)
  {
    idle_level[ch] = gpio_get_level(channel[ch].pin);
    Serial.printf("  %s (GPIO %d) %s", channel[ch].name, (int)channel[ch].pin,
                  idle_level[ch] ? "high" : "low");
  }
  Serial.printf("\r\nWaiting for a bang, times are relative to North\r\n");
  arm();
}

void loop(void)
{
  if ( cap_mask != 0 )
  {
    delay(WINDOW_MS);
    report();
    wait_for_silence();
    arm();
  }
}
