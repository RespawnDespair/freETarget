/*
 * capture_test.ino
 *
 * Proof of concept for a freETarget minimalistic port to the ESP32.
 *
 * Question it answers: can the ESP32 MCPWM capture hardware timestamp four
 * microphone edges without jitter, also across the two MCPWM groups and
 * with WiFi running?
 *
 * No wiring needed: one GPIO is driven by this sketch and routed through the
 * GPIO matrix to every capture channel at once. All channels therefore see
 * the same edge, so any spread in the differences is timer jitter.
 *
 * Expected result:
 *   - channels in the same group: difference exactly 0 ticks
 *   - channels in the other group: a constant offset (the two capture timers
 *     start at different moments), spread 0 ticks
 *
 * Board: ESP32-S3 Dev Module (ESP32-S3-WROOM-1) or ESP32 Dev Module
 */
#include <WiFi.h>
#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"

#define TEST_PIN      GPIO_NUM_4        // Driven and captured, leave unconnected
#define N_GROUPS      2
#define CH_PER_GROUP  3
#define N_CHANNELS    (N_GROUPS * CH_PER_GROUP)
#define N_PULSES      2000
#define ALL_CAPTURED  ((1u << N_CHANNELS) - 1)

static mcpwm_cap_timer_handle_t   cap_timer[N_GROUPS];
static mcpwm_cap_channel_handle_t cap_chan[N_CHANNELS];
static volatile uint32_t cap_value[N_CHANNELS];
static volatile uint32_t cap_mask;
static uint32_t resolution_hz;

static bool IRAM_ATTR on_capture(mcpwm_cap_channel_handle_t chan, const mcpwm_capture_event_data_t *edata, void *user)
{
  uint32_t ch = (uint32_t)(uintptr_t)user;

  if ( (cap_mask & (1u << ch)) == 0 )   // Keep the first edge only, as the Mega ISRs do
  {
    cap_value[ch] = edata->cap_value;
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

    for (int c = 0; c != CH_PER_GROUP; c++)
    {
      int ch = g * CH_PER_GROUP + c;
      mcpwm_capture_channel_config_t chan_config = {};
      chan_config.gpio_num       = TEST_PIN;
      chan_config.prescale       = 1;
      chan_config.flags.pos_edge = true;
      if ( !check("new_capture_channel", mcpwm_new_capture_channel(cap_timer[g], &chan_config, &cap_chan[ch])) )
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

    if ( !check("timer_enable", mcpwm_capture_timer_enable(cap_timer[g]))
      || !check("timer_start", mcpwm_capture_timer_start(cap_timer[g])) )
    {
      return false;
    }
  }

  check("get_resolution", mcpwm_capture_timer_get_resolution(cap_timer[0], &resolution_hz));

/*
 * Creating the channels left the pad as an input. Enable the output driver
 * as well, the input routing to the capture channels stays in place.
 */
  gpio_set_level(TEST_PIN, 0);
  return check("set_direction", gpio_set_direction(TEST_PIN, GPIO_MODE_INPUT_OUTPUT));
}

static void run_phase(const char *name)
{
  int32_t  d_min[N_CHANNELS], d_max[N_CHANNELS];
  uint32_t missed = 0;

  for (int ch = 0; ch != N_CHANNELS; ch++)
  {
    d_min[ch] = INT32_MAX;
    d_max[ch] = INT32_MIN;
  }

  for (int i = 0; i != N_PULSES; i++)
  {
    cap_mask = 0;
    gpio_set_level(TEST_PIN, 1);

    uint32_t start = micros();
    while ( (cap_mask != ALL_CAPTURED) && ((micros() - start) < 10000) )
    {
    }
    gpio_set_level(TEST_PIN, 0);

    if ( cap_mask != ALL_CAPTURED )
    {
      missed++;
    }
    else
    {
      for (int ch = 1; ch != N_CHANNELS; ch++)
      {
        int32_t d = (int32_t)(cap_value[ch] - cap_value[0]);
        if ( d < d_min[ch] ) d_min[ch] = d;
        if ( d > d_max[ch] ) d_max[ch] = d;
      }
    }
    delay(2);
  }

  double ns_per_tick = 1e9 / (double)resolution_hz;
  Serial.printf("\r\n%s: %d pulses, %lu missed\r\n", name, N_PULSES, (unsigned long)missed);
  for (int ch = 1; ch != N_CHANNELS; ch++)
  {
    int32_t spread = d_max[ch] - d_min[ch];
    Serial.printf("  ch%d (group %d) - ch0: offset %ld ticks, spread %ld ticks = %.1f ns\r\n",
                  ch, ch / CH_PER_GROUP, (long)d_min[ch], (long)spread, spread * ns_per_tick);
  }
}

void setup(void)
{
  Serial.begin(115200);
  delay(1000);
  Serial.printf("\r\nfreETarget ESP32 capture test\r\n");

  if ( !init_capture() )
  {
    Serial.printf("Capture setup failed, stopping\r\n");
    return;
  }
  Serial.printf("Capture resolution: %lu Hz = %.2f ns per tick\r\n",
                (unsigned long)resolution_hz, 1e9 / (double)resolution_hz);

  run_phase("WiFi off");

  WiFi.softAP("freETarget-test", "freetarget");
  delay(1000);
  run_phase("WiFi access point on");

  Serial.printf("\r\nDone\r\n");
}

void loop(void)
{
  delay(1000);
}
