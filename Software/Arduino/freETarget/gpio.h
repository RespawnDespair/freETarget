/*----------------------------------------------------------------
 *
 * gpio.h
 *
 * Header file for GPIO functions
 *
 *---------------------------------------------------------------*/
#ifndef _GPIO_H_
#define _GPIO_H_

/*
 * Global functions
 */
void init_gpio(void);                                     // Initialize the GPIO ports
void arm_timers(void);                                    // Make the board ready
void clear_running(void);                                 // Clear the run flip flop 
unsigned int is_running(void);                            // Return a bit mask of running sensors 
void set_LED(int state_RDY, int state_X, int state_y);    // Manage the LEDs
unsigned int read_DIP(unsigned int dip_mask);             // Read the DIP switch register
unsigned int read_counter(unsigned int direction);
void stop_timers(void);                                   // Turn off the counter registers
void trip_timers(void);
bool read_in(unsigned int port);                          // Read the selected port
void read_timers(unsigned int* timer_counts);             // Read and return the counter registers
void drive_paper(void);                                   // Turn on the paper motor
void enable_face_interrupt();                             // Turn on the face strike interrupt
void disable_face_interrupt(void);                        // Turn off the face strike interrupt
void enable_sensor_interrupt();                           // Turn on the sensor interrupt
void disable_sensor_interrupt(void);                      // Turn off the sensor strike interrupt
void multifunction_init(void);                            // Initialize the multifunction switches
void multifunction_switch(void);                          // Handle the actions of the DIP Switch signal
void multifuction_display(void);                          // Display the MFS settings
void multifunction_wait_open(void);                       // Wait for both multifunction switches to be open
void output_to_all(char* s);                              // Multipurpose driver
void char_to_all(char ch);                                // Output a single character
void digital_test(void);                                  // Execute the digital test
void paper_on_off(bool on);                               // Turn the motor on or off
void rapid_green(unsigned int state);                     // Drive the GREEN light
void rapid_red(unsigned int state);                       // Drive the RED light
char get_all(void);                                       // Read from all of the ports
char aux_spool_read(void);                                // Read something from the AUX spool
int  aux_spool_available(void);                           // Is there something in the AUX spool
void aux_spool_put(char ch);                              // Put something into the  AUX spool
char json_spool_read(void);                               // Read something from the AUX spool
int  json_spool_available(void);                          // Is there something in the AUX spool
void json_spool_put(char ch);                             // Put something into the  AUX spool

/*
 *  Port Definitions
 */


#define RUN_PORT    PINA

#define DIP_0        9
#define RED_OUT      9                  // Rapid fire RED on DIP0
#define DIP_1       10
#define DIP_2       11
#define DIP_3       12
#define GREEN_OUT   12                  // Rapid fire GREEN on DIP3

#define RED_MASK     1                  // Use DIP 0
#define GREEN_MASK   8                  // Use DIP 3

/*
 * Multifunction Switch Use when using DIP Switch for MFS
 */
#define HOLD1(x)    LO10((x))          // Low digit        xxxx2
#define HOLD2(x)    HI10((x))          // High digit       xxx2x
#define TAP1(x)     HLO10((x))         // High Low digit   xx2xx
#define TAP2(x)     HHI10((x))         // High High digit  x2xxx
#define HOLD12(x)   HHH10((x))         // Highest digit    2xxxx

/*
 * DIP Switch Use. 
 */
//                      From DIP                   From Software
#define CALIBRATE       ((digitalRead(DIP_3) == 0)    + 0)   // 1 Go to Calibration Mode
#define DIP_SW_A        ((digitalRead(DIP_2) == 0)    + 0)   // 2 When CALIBRATE is asserted, use lower trip point
#define CAL_LOW         (DIP_SW_A)
#define DIP_SW_B        ((digitalRead(DIP_1) == 0)    + 0)   // 4 When CALIBRATE is asserted, use higher trip point
#define CAL_HIGH        (DIP_SW_B)
#define VERBOSE_TRACE   ((digitalRead(DIP_0) == 0)    + 0)   // 8 Show the verbose software trace

#define VSET_PWM     8          // VREF setting
#define CTS_U        7
#define RTS_U        6
#define LED_PWM      5          // PWM Port
#define LED_RDY      4
#define LED_X        22         //ToDo add in description, was 3
#define LED_Y        2
#define LON          1          // Turn the LED on
#define LOF          0          // Turn the LED off
#define LXX         -1          // Leave the LED alone
#define L(A, B, C)  (A), (B), (C)

#define NORTH        0
#define EAST         1
#define SOUTH        2
#define WEST         3
#define TRIP_NORTH   0x01
#define TRIP_EAST    0x02
#define TRIP_SOUTH   0x04
#define TRIP_WEST    0x08
#define TIMER_NORTH  1
#define TIMER_EAST   3
#define TIMER_SOUTH  4
#define TIMER_WEST   5


#define PAPER        18                    // Paper advance drive active low (TX1)
#define PAPER_ON      0
#define PAPER_OFF     1
#define PAPER_ON_300  1
#define PAPER_OFF_300 0

#define FACE_SENSOR  19

#define SPARE_1      22
#define POWER_TAP     0                   // DIP A/B used to wake up
#define PAPER_FEED    1                   // DIP A/B used as a paper feed
#define LED_ADJUST    2                   // DIP A/B used to set LED brightness
#define PAPER_SHOT    3                   // DIP A/B Advance paper one cycle
#define PC_TEST       4                   // DIP A/B used to trigger fake shot
#define ON_OFF        5                   // DIP A/B used to turn the target ON or OFF
#define MFS_SPARE_6   6
#define MFS_SPARE_7   7
#define MFS_SPARE_8   8
#define TARGET_TYPE   9                   // Sent target type with score

#define NO_ACTION     0                   // DIP usual function
#define RAPID_RED     1                   // Rapid Foir Red Output
#define RAPID_GREEN   2                   // Rapid Fire Green Output

#define J10_1      VCC
#define J10_2       14                    // TX3
#define J10_3       15                    // RX3
#define J10_4       19                    // RX1
#define J10_5       18                    // TX1
#define J10_6      GND



#define EOF 0xFF

#endif
