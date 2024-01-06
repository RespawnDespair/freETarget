/*-------------------------------------------------------
 * 
 * gpio.ino
 * 
 * General purpose GPIO driver
 * 
 * ----------------------------------------------------*/

#include "timer.h"

const GPIO init_table[] = {   
  
  {DIP_0,       "\"DIP_0\":",    INPUT_PULLUP, 0},
  {DIP_1,       "\"DIP_1\":",    INPUT_PULLUP, 0},
  {DIP_2,       "\"DIP_2\":",    INPUT_PULLUP, 0},
  {DIP_3,       "\"DIP_3\":",    INPUT_PULLUP, 0},

  {PD4,         "\"ICP_1\":",    INPUT_PULLUP, 0 }, // N 38 connected to PD7, PD6, PD5
  {PE7,         "\"ICP_3\":",    INPUT_PULLUP, 0 }, // E 3 conencted to PE6, PE5
  {PL0,         "\"ICP_4\":",    INPUT_PULLUP, 0 }, // S 49
  {PL1,         "\"ICP_5\":",    INPUT_PULLUP, 0 }, // W 48

  {PD7,         "\"ICP_1_\":",    INPUT, 0 }, // set parallel pins to high z
  {PD6,         "\"ICP_1_\":",    INPUT, 0 },
  {PD5,         "\"ICP_1_\":",    INPUT, 0 },
  {PE6,         "\"ICP_3_\":",    INPUT, 0 },
  {PE5,         "\"ICP_3_\":",    INPUT, 0 },
    

  {LED_RDY,     "\"RDY\":",      OUTPUT, 1},
  {LED_X,       "\"X\":",        OUTPUT, 1},
  {LED_Y,       "\"Y\":",        OUTPUT, 1},

  {LED_PWM,     "\"LED_PWM\":",  OUTPUT, 0},
  {VSET_PWM,    "\"VSET_PWM\":", OUTPUT, 0},
  {RTS_U,       "\"RTS_U\":",    OUTPUT, 1},
  {CTS_U,       "\"CTS_U\":",    INPUT_PULLUP, 0},

  {FACE_SENSOR, "\"FACE\":",     INPUT_PULLUP, 0},
  
  {PAPER,      "\"PAPER\":",     OUTPUT, 1},               // 18-Paper drive active low
  
  {EOF, EOF, EOF, EOF} };


void face_ISR(void);                      // Acknowledge a face strike
void sensor_ISR(void);                    // Begin recording times for a target shot

static bool fcn_DIP_SW_A(void);           // Function to read DIP_SW_A
static bool fcn_DIP_SW_B(void);           // Function to read DIP_SW_B
static void sw_state (bool* (fcn_state)(void), unsigned long*  which_timer, void* (fcn_action)(void)); // Do something with the switches
static void send_fake_score(void);        // Send a fake score to the PC

static unsigned int dip_mask;             // Used if the MFS2 uses the DIP_0 or DIP_3
static char aux_spool[128];               // Spooling buffer from the AUX port
static char json_spool[64];               // Spool for JSON
static unsigned int  aux_spool_in, aux_spool_out; // Pointer to the spool
static unsigned int  json_spool_in, json_spool_out; // Pointer to the spool
static uint16_t T[4];                     // saves the time values


/*-----------------------------------------------------
 * 
 * function: gpio_init
 * 
 * brief: Initalize the various GPIO ports
 * 
 * return: None
 * 
 *-----------------------------------------------------
 *
 * The GPIO programming is held in a stgrucutre and 
 * copied out to the hardware on power up.
 *-----------------------------------------------------*/

void init_gpio(void)
{
  int i;

  if ( DLT(DLT_CRITICAL) ) 
  {
    Serial.print(T("init_gpio()"));  
  }
  
  i = 0;
  while (init_table[i].port != 0xff )
  {
    pinMode(init_table[i].port, init_table[i].in_or_out);
    if ( init_table[i].in_or_out == OUTPUT )
    {
      digitalWrite(init_table[i].port, init_table[i].value);
    }
    i++;
  }
  
  multifunction_init();
  disable_face_interrupt();
  set_LED_PWM(0);             // Turn off the illumination for now
  
/*
 * Special case of the witness paper
 */
  pinMode(PAPER, OUTPUT);
  paper_on_off(false);        // Turn it off
  
/*
 * All done, return
 */  
  return;
}


/*-----------------------------------------------------
 * 
 * function: read_counter
 * 
 * brief: Read specified counter register
 * 
 * return: 16 bit counter register
 * 
 *-----------------------------------------------------*/


unsigned int read_counter
  (
  unsigned int direction         // What direction are we reading?
  )
{
  // return timer value
  return T[direction];
}

/*-----------------------------------------------------
 * 
 * function: is_running
 * 
 * brief: Determine if the clocks are running
 * 
 * return: TRUE if any of the counters are running
 * 
 *-----------------------------------------------------*/

unsigned int is_running (void)
{
  unsigned int i;
  
  i = (TCCR1B & 1) +  ((TCCR3B & 1) << 1) + ((TCCR4B & 1) << 2) + ((TCCR5B & 1) <<3);

 /*
  *  Return the running mask
  */
  return i;
  }

/*-----------------------------------------------------
 * 
 * function: arm_timers
 * 
 * brief: Strobe the control lines to start a new cycle
 * 
 * return: NONE
 * 
 *-----------------------------------------------------
 *
 * The counters are armed by
 * 
 *   Stopping the current cycle
 *   Taking the counters out of read
 *   Making sure the oscillator is running
 *   Clearing the counters
 *   Enabling the counters to run again
 * 
 *-----------------------------------------------------*/
void arm_timers(void)
{
  // Init TCCRxA
  TCCR1A = 0;
  TCCR3A = 0;
  TCCR4A = 0;
  TCCR5A = 0;

  //clear interrrupts
  TIFR1 |= (1 << ICF1);
  TIFR3 |= (1 << ICF3);
  TIFR4 |= (1 << ICF4);
  TIFR5 |= (1 << ICF5);

  // Init TIMSKx: 7, 6, 5 - ICU, 4, 3, 2, 1, 0 - OVF
  TIMSK1 = B00100000;  // Enable Timer OVF & CAPT Interrupts
  TIMSK3 = B00100000;
  TIMSK4 = B00100000;
  TIMSK5 = B00100000;

  // offset due to delayed start
  TCNT1 = 0;
  TCNT3 = 2;
  TCNT4 = 4;
  TCNT5 = 6;

  // TCCRxB: 7 - noise canceling, 6 - edge (0 falling), 5, 4, 3, 2:0 - prescaler (1 system clock)
  // 16 MHz clock, 62.5 ns/count
  TCCR1B = B10000001;  // Internal Clock, Prescaler = 1, ICU Filter DE, ICU Pin falling
  TCCR3B = B10000001;
  TCCR4B = B10000001;
  TCCR5B = B10000001;
 
  return;
}

void clear_running(void)          // Reset the RUN flip Flop
{
  return;
}

/*
 *  Stop the oscillator
 */
void stop_timers(void)
{
  TCCR1B = B00000000;  
  TCCR3B = B00000000;
  TCCR4B = B00000000;
  TCCR5B = B00000000;
  return;
}

/*
 *  Trip the counters for a self test
 */
void trip_timers(void)
{
  // not needed any longer
  return;
}

/*-----------------------------------------------------
 * 
 * function: enable_face_interrupt
 * function: disable_face_interrupt
 * 
 * brief: Turn on the face detection interrupt
 * 
 * return: NONE
 * 
 *-----------------------------------------------------
 *
 * This enables the face interrupts so that shot detection
 * an be perfomed as a thread and not inline with the code
 * 
 *-----------------------------------------------------*/
void enable_face_interrupt(void)
{
  if ( revision() >= REV_300 )
  {
    attachInterrupt(digitalPinToInterrupt(FACE_SENSOR),  face_ISR, CHANGE);
  }

  return;
}

void disable_face_interrupt(void)
{
  if ( revision() >= REV_300 )
  {
    detachInterrupt(digitalPinToInterrupt(FACE_SENSOR));
  }

  return;
}

/*-----------------------------------------------------
 * 
 * function: read_DIP
 * 
 * brief: READ the jumper block setting
 * 
 * return: TRUE for every position with a jumper installed
 * 
 *-----------------------------------------------------
 *
 * The DIP register is read and formed into a word.
 * The word is complimented to return a 1 for every
 * jumper that is installed.
 * 
 * OR in the json_dip_switch to allow remote testing
 * OR in  0xF0 to allow for compile time testing
 *-----------------------------------------------------*/
unsigned int read_DIP
(
  unsigned int dip_mask
)
{
  unsigned int return_value;
  
  if ( revision() < REV_300 )          // The silkscreen was reversed in Version 3.0  oops
  {
    return_value =  (~((digitalRead(DIP_0) << 0) + (digitalRead(DIP_1) << 1) + (digitalRead(DIP_2) << 2) + (digitalRead(DIP_3) << 3))) & 0x0F;  // DIP Switch
  }
  else
  {
    return_value =  (~((digitalRead(DIP_3) << 0) + (digitalRead(DIP_2) << 1) + (digitalRead(DIP_1) << 2) + (digitalRead(DIP_0) << 3))) & 0x0F;  // DIP Switch
  }
  return_value &= (~dip_mask);
  return_value |= json_dip_switch;  // JSON message
  return_value |= 0xF0;             // COMPILE TIME

  return return_value;
}  

/*-----------------------------------------------------
 * 
 * function: set_LED
 * 
 * brief:    Set the state of all the LEDs
 * 
 * return:   None
 * 
 *-----------------------------------------------------
 *
 * The state of the LEDs can be turned on or off 
 * 
 * -1 '-' Leave alone
 *  0 '.' Turn LED off
 *  1 '*' Turn LED on
 *  
 *  The macro L(RDY, X, Y) defines 
 * 
 *-----------------------------------------------------*/
void set_LED
  (
    int state_RDY,        // State of the Rdy LED
    int state_X,          // State of the X LED
    int state_Y           // State of the Y LED
    )
{ 
  switch (state_RDY)
  {
    case 0:
    case '.':
        digitalWrite(LED_RDY, 1 );
        break;
    
    case 1:
    case '*':
        digitalWrite(LED_RDY, 0 );
        break;
  }
  
  switch (state_X)
  {
    case 0:
    case '.':
        digitalWrite(LED_X, 1 );
        break;
    
    case 1:
    case '*':
        digitalWrite(LED_X, 0 );
        break;
  }

  switch (state_Y)
  {
    case 0:
    case '.':
        digitalWrite(LED_Y, 1 );
        break;
    
    case 1:
    case '*':
        digitalWrite(LED_Y, 0 );
        break;
  }
  return;  
  }

/* 
 *  HAL Discrete IN
 */
bool read_in(unsigned int port)
{
  return digitalRead(port);
}

/*-----------------------------------------------------
 * 
 * function: read_timers
 * 
 * brief:   Read the timer registers, compensating fo
 *          timer overrun
 *          fixing the first time to 20000
 * 
 * return:  All four timer registers read and stored
 * 
 *-----------------------------------------------------*/
void read_timers
  (
    unsigned int* timer_ptr
  )
{
  unsigned int i;

  long T_correct;
  long T_diff;
  
  *(timer_ptr + 0) = 20000;
  
  for (i=1; i<=3; i++)
  {
  
    T_diff = (long)T[0]-long(T[i]);
    if (T_diff > 10000)
    {
        T_correct = T_diff - 45536;
    }
    else if (T_diff < - 10000)
    {
        T_correct = T_diff + 85536;
    }
    else
    {
        T_correct = T_diff + 20000;
    }


    
    // Serial.print("-");
    // Serial.println(T_correct);
  
    *(timer_ptr + i) = T_correct;
  }

  return;
}

/*-----------------------------------------------------
 * 
 * function: drive_paper
 * 
 * brief:    Turn on the witness paper motor for json_paper_time
 * 
 * return:  None
 * 
 *-----------------------------------------------------
 *
 * The function turns on the motor for the specified
 * time.  The motor is cycled json_paper_step times
 * to drive a stepper motor using the same circuit.
 * 
 * Use an A4988 to drive te stepper in place of a DC
 * motor
 * 
 * There is a hardare change between Version 2.2 which
 * used a transistor and 3.0 that uses a FET.
 * The driving circuit is reversed in the two boards.
 * 
 * DC Motor
 * Step Count = 0
 * Step Time = 0
 * Paper Time = Motor ON time
 *
 * Stepper
 * Paper Time = 0
 * Step Count = X
 * Step Time = Step On time
 * 
 * {"PAPER":1, "DELAY":1, "PAPER":0}
 *-----------------------------------------------------*/

void drive_paper(void)
{
  unsigned long s_time, s_count;              // Step time and count
  volatile unsigned long paper_time;

/*
 * Set up the count or times based on whether a DC or stepper motor is used
 */

  s_time = json_paper_time;                       // On time.
  if ( json_step_time != 0 )                      // Non-zero means it's a stepper motor
  {
    s_time = json_step_time;                      // the one we use
  }

  s_count = 1;                                    // Default to one cycle (DC or Stepper Motor)
  if ( json_step_count != 0 )                     // Non-zero means it's a stepper motor
  {
    s_count = json_step_count;                    // the one we use
  }

  if ( s_time == 0 )                              // Nothing to do if the time is zero.
  {
    return;
  }
  
  if ( DLT(DLT_INFO) )
  {
    Serial.print(T("Advancing paper: ")); Serial.print(s_time); Serial.print(T("ms  Steps: ")); Serial.print(s_count);
  }

/*
 * Drive the motor on and off for the number of cycles
 * at duration
 */
  timer_new(&paper_time, s_time);         // Create the timer

  while ( s_count )
  {
    paper_on_off(true);                 // Motor ON
    paper_time = s_time; 
    while ( paper_time != 0 )
    {    
      continue;
    }
    paper_on_off(false);                  // Motor OFF
    paper_time = 5;
    while ( paper_time )
    {
      continue;
    }
    s_count--;                            // Repeat for the steps
  }

  timer_delete(&paper_time);              // Finished with the timer
  
 /*
  * All done, return
  */
  return;
 }

/*-----------------------------------------------------
 * 
 * function: paper_on_off
 * 
 * brief:    Turn the withness paper motor on or off
 * 
 * return:  None
 * 
 *-----------------------------------------------------
 *
 * The witness paper motor changed polarity between 2.2
 * and Version 3.0.
 * 
 * This function reads the board revision and controls 
 * the FET accordingly
 * 
 *-----------------------------------------------------*/

static void paper_on_off                        // Function to turn the motor on and off
  (
  bool on                                       // on == true, turn on motor drive
  )
{
  if ( DLT(DLT_INFO) )
  {
    Serial.print("paper_on_off("); Serial.print(on); Serial.print(T(")"));
  }
  
  if ( on == true )
  {
    if ( revision() < REV_300 )                 // Rev 3.0 changed the motor sense
    {
      digitalWrite(PAPER, PAPER_ON);            // Turn it on
    }
    else
    {
      digitalWrite(PAPER, PAPER_ON_300);        //
    }
  }
  else
  {
    if ( revision() < REV_300 )                 // Rev 3.0 changed the motor sense
    {
      digitalWrite(PAPER, PAPER_OFF);            // Turn it off
    }
    else
    {
      digitalWrite(PAPER, PAPER_OFF_300);        //
    }
  }

/*
 * No more, return
 */
  return;
}


/*-----------------------------------------------------
 * 
 * function: face_ISR
 * 
 * brief:    Face Strike Interrupt Service Routint
 * 
 * return:   None
 * 
 *-----------------------------------------------------
 *
 * Sensor #5 is attached to the digital input #19 and
 * is used to generate an interrrupt whenever a face
 * strike has been detected.
 * 
 * The ISR simply counts the number of cycles.  Anything
 * above 0 is an indication that sound was picked up
 * on the front face.
 * 
 *-----------------------------------------------------*/
 void face_ISR(void)
 {
  face_strike++;      // Got a face strike

  if ( DLT(DLT_INFO) )
  {
    Serial.print(T("\r\nface_ISR():")); Serial.print(face_strike);
  }

  return;
 }

 /*
 * Common function to indicate a fault // Cycle LEDs 5x
 */
void blink_fault
  (                                        
  unsigned int fault_code                 // Fault code to blink
  )
{
  unsigned int i;

  for (i=0; i != 3; i++)
  {
    set_LED(fault_code & 4, fault_code & 2, fault_code & 1);  // Blink the LEDs to show an error
    delay(ONE_SECOND/4);
    fault_code = ~fault_code;
    set_LED(fault_code & 4, fault_code & 2, fault_code & 1);                    // Blink the LEDs to show an error
    delay(ONE_SECOND/4);
    fault_code = ~fault_code;
  }

 /*
  * Finished
  */
  return;
}

/*-----------------------------------------------------
 * 
 * function: multifunction_init
 * 
 * brief:    Use the multifunction switches during starup
 * 
 * return:   None
 * 
 *-----------------------------------------------------
 * 
 * Read the jumper header and modify the initialization
 * 
 *-----------------------------------------------------*/
 void multifunction_init(void)
 {
  unsigned int dip;

/*
 * Check to see if the DIP switch has been overwritten
 */
  if ( (HOLD1(json_multifunction2) == RAPID_RED) 
        || (HOLD1(json_multifunction2) == RAPID_GREEN))
  {
      pinMode(DIP_0, OUTPUT);
      digitalWrite(DIP_0, 1);
      dip_mask = RED_MASK;
  }

  if (  (HOLD2(json_multifunction2) == RAPID_RED)
      || (HOLD2(json_multifunction2) == RAPID_GREEN ) )
  {
      pinMode(DIP_3, OUTPUT);
      digitalWrite(DIP_3, 1);
      dip_mask |= GREEN_MASK;
  }

/*
 * Continue to read the DIP switch
 */
  dip = read_DIP(dip_mask) & 0x0f;      // Read the jumper header

  if ( dip == 0 )                       // No jumpers in place
  { 
    return;                             // Carry On
  }


  if ( DIP_SW_A && DIP_SW_B )           // Both switches closed?
  {
    factory_nonvol(false);              // Initalize the nonvol but do not calibrate
  }

  else
  {
    if ( DIP_SW_A )                     // Switch A pressed
    {
      is_trace = 10;                    // Turn on tracing
    }
  
    if ( DIP_SW_B )                     // Switch B pressed
    {

    }
  }
  
/*
 * The initialization override has been finished
 */
  return;
}

 
/*-----------------------------------------------------
 * 
 * function: multifunction_switch
 * 
 * brief:    Carry out the functions of the multifunction switch
 * 
 * return:   Switch state
 * 
 *-----------------------------------------------------
 *
 * The actions of the DIP switch will change depending on the 
 * mode that is programmed into it.
 * 
 * For some of the DIP switches, tapping the switch
 * turns the LEDs on, and holding it will carry out 
 * the alternate activity.
 * 
 * MFS_TAP1\": \"%s\",\n\r\"MFS_TAP2\": \"%s\",\n\r\"MFS_HOLD1\": \"%s\",\n\r\"MFS_HOLD2\": \"%s\",\n\r\"MFS_HOLD12\": \"%s\",\n\r", 
 * Special Cases
 * 
 * Both switches pressed, Toggle the Tabata State
 * Either switch set for target type switch
 *-----------------------------------------------------*/
                           
void multifunction_switch(void)
 {
    unsigned int  action;               // Action to happen
    unsigned int  i;                    // Iteration Counter
    unsigned long now;
    
    if ( CALIBRATE )
    {
      return;                           // Not used if in calibration mode
    }

/*
 * Figure out what switches are pressed
 */
   action = 0;                         // No switches pressed
   if ( DIP_SW_A != 0 )
   {
     action += 1;                     // Remember how we got here
   }
   if ( DIP_SW_B != 0 )
   {
     action += 2;
   }

/*
 * Special case of a target type, ALWAyshot_mm process this switch even if it is closed
 */
   if ( HOLD1(json_multifunction) == TARGET_TYPE ) 
   {
     sw_state(HOLD1(json_multifunction));
     action &= ~1;
     action += 4;
   }
   else if ( HOLD2(json_multifunction) == TARGET_TYPE ) 
   {
     sw_state(HOLD2(json_multifunction));
     action &= ~2;
     action += 8;
   }

   if ( action == 0 )                 // Nothing to do
   {
     return;
   }
   
/*
 * Delay for one second to detect a tap
 * Check to see if the switch has been pressed for the first time
 */
  timer_new(&now, ONE_SECOND);
  while ( now )
  {
    if ( DIP_SW_A )
    {
      set_LED(L('-', '*', '-'));
    }
    else
    {
      set_LED(L('-', '.', '-'));
    }
    if ( DIP_SW_B )
    {
      set_LED(L('-', '-', '*'));
    }
    else
    {
      set_LED(L('-', '-', '.'));
    }
  }
  timer_delete(&now);
  
  if ( (DIP_SW_A == 0 )
        && (DIP_SW_B == 0 ) )             // Both switches are open? (tap)
   {
      if ( action & 1 )
      {
        sw_state(TAP1(json_multifunction));
      }
      if ( action & 2 )
      {
        sw_state(TAP2(json_multifunction));
      }
   }
   
/*
 * Look for the special case of both switches pressed
 */
  if ( (DIP_SW_A) && (DIP_SW_B) )         // Both pressed?
  {
    sw_state(HOLD12(json_multifunction));
  }
      
/*
 * Single button pressed manage the target based on the configuration
 */
  else
  {
    if ( DIP_SW_A )
    {
      sw_state(HOLD1(json_multifunction));
    }
    if ( DIP_SW_B )
    {
      sw_state(HOLD2(json_multifunction));
    }
  }
  
/*
 * All done, return the GPIO state
 */
  multifunction_wait_open();      // Wait here for the switches to be open

  set_LED(LED_READY);
  return;
}


/*-----------------------------------------------------
 * 
 * function: multifunction_switch helper functions
 * 
 * brief:    Small functioins to work with the MFC
 * 
 * return:   Switch state
 * 
 *-----------------------------------------------------
 *
 * The MFC software above has been organized to use helper
 * functions to simplify the construction and provide
 * consistency in the operation.
 * 
 *-----------------------------------------------------*/

/*
 * Carry out an action based on the switch state
 */
static void sw_state 
    (
    unsigned int action
    )
{     
  
  char s[128];                          // Holding string 

  if ( DLT(DLT_DIAG) )
  {
    Serial.print(T("Switch action: ")); Serial.print(action);
  }

  switch (action)
  {
    case POWER_TAP:
      set_LED_PWM_now(json_LED_PWM);      // Yes, a quick press to turn the LED on
      delay(ONE_SECOND/2),
      set_LED_PWM_now(0);                 // Blink
      delay(ONE_SECOND/2);
      set_LED_PWM_now(json_LED_PWM);      // and leave it on
      power_save = (long)json_power_save * 60L * (long)ONE_SECOND; // and resets the power save time
      json_power_save += 30;      
      sprintf(s, "\r\n{\LED_PWM\": %d}\n\r", json_power_save);
      output_to_all(s);  
        break;
        
    case PAPER_FEED:                      // The switch acts as paper feed control
      paper_on_off(true);                 // Turn on the paper drive
      while ( (DIP_SW_A || DIP_SW_B) )    // Keep it on while the switches are pressed 
      {
        continue; 
      }
      paper_on_off(false);                // Then turn it off
      break;

   case PAPER_SHOT:                       // The switch acts as paper feed control
      drive_paper();                      // Turn on the paper drive
      while ( (DIP_SW_A || DIP_SW_B) )    // Keep it on while the switches are pressed 
      {
        continue; 
      }
      break;
      
   case PC_TEST:                         // Send a fake score to the PC
      send_fake_score();
      break;
      
   case ON_OFF:                          // Turn the target off
      bye();                              // Stay in the Bye state until a wake up event comes along
      break;
      
    case LED_ADJUST:
      json_LED_PWM += 10;                 // Bump up the LED by 10%
      if ( json_LED_PWM > 100 )
      {
        json_LED_PWM = 0;                 // Force to zero on wrap around
      }
      set_LED_PWM_now(json_LED_PWM);      // Set the brightness
      EEPROM.put(NONVOL_LED_PWM, json_LED_PWM);   
      sprintf(s, "\r\n{\LED_BRIGHT\": %d}\n\r", json_LED_PWM);
      output_to_all(s);  
      break;

    case TARGET_TYPE:                     // Over ride the target type if the switch is closed
      json_target_type = 0;
      if (HOLD1(json_multifunction) == TARGET_TYPE) // If the switch is set for a target type
      {
        if ( DIP_SW_A )
        {
          json_target_type = 1;           // 
        }
      }
      if (HOLD2(json_multifunction) == TARGET_TYPE) 
      {
        if ( DIP_SW_B )
        {
          json_target_type = 1;
        }
      }
      break;
      
    default:
      break;
  }



/*
 * All done, return
 */
  return;
}

/*
 * Wait here for the switches to be opened
 */
void multifunction_wait_open(void)
{
  while (1)
  {
    if ( (DIP_SW_A == 0 )
        && (DIP_SW_B == 0) ) 
    {
      return;
    }

    if ( (HOLD1(json_multifunction) == TARGET_TYPE ) 
      && (DIP_SW_B == 0) )
    {
      return;
    }
    
    if ( ( HOLD2(json_multifunction) == TARGET_TYPE) 
      && ( DIP_SW_A == 0 ) )
    {
      return;
    }
  }
  
  return;
}

/*-----------------------------------------------------
 * 
 * function: multifunction_display
 * 
 * brief:    Display the MFS settings as text
 * 
 * return:   None
 * 
 *-----------------------------------------------------
 *
 * The MFS is encoded as a 3 digit packed BCD number
 * 
 * This function unpacks the numbers and displayes it as
 * text in a JSON message.
 * 
 *-----------------------------------------------------*/
 //                             0            1            2             3            4             5            6    7    8          9
static char* mfs_text[] = { "WAKE_UP", "PAPER_FEED", "ADJUST_LED", "PAPER_SHOT", "PC_TEST",  "POWER_ON_OFF",   "6", "7", "8", "TARGET_TYPE"};

 //                              0           1            2             3            4             5            6    7    8          9
static char* mfs2_text[] = { "DEFAULT", "RAPID_RED", "RAPID_GREEN",    "3",         "4",          "5",   "      6", "7", "8",       "9"};

void multifunction_display(void)
{
  char s[128];                          // Holding string

  sprintf(s, "\"MFS_TAP1\": \"%s\",\n\r\"MFS_TAP2\": \"%s\",\n\r\"MFS_HOLD1\": \"%s\",\n\r\"MFS_HOLD2\": \"%s\",\n\r\"MFS_HOLD12\": \"%s\",\n\r", 
  mfs_text[TAP1(json_multifunction)], mfs_text[TAP2(json_multifunction)], mfs_text[HOLD1(json_multifunction)], mfs_text[HOLD2(json_multifunction)], mfs_text[HOLD12(json_multifunction)]);
  output_to_all(s);  

  sprintf(s, "\"MFS_CONFIG\": \"%s\",\n\r\"MFS_DIAG\": \"%s\",\n\r", 
  mfs2_text[HOLD1(json_multifunction2)], mfs2_text[HOLD2(json_multifunction2)]);
  output_to_all(s);  
  
/*
 * All done, return
 */
  return;
}

/*-----------------------------------------------------
 * 
 * function: get_all
 * 
 * brief:    Read all of the serial ports
 * 
 * return:   Next character in the serial port
 * 
 *-----------------------------------------------------
 *
 * Each of the available serial ports are polled for
 * a character.
 * 
 * If a token ring is enabled, the get_all() function is 
 * uses the JSON spool
 * 
 *-----------------------------------------------------*/
char get_all(void) 
{
  char ch;

  if ( DISPLAY_SERIAL.available() )                   // Display port
  {
    ch =  DISPLAY_SERIAL.read();
    return ch;
  }

  if ( Serial.available() )                         // Check the serial port
  {
    ch = Serial.read();
      return ch;
  }
  
  switch (json_token)
  {
    case TOKEN_WIFI:
      if ( esp01_available() )
      {
        ch = esp01_read();
        return ch;                                      // ESP01 only avaialbe in WIFI
      }
      break;

    case TOKEN_SLAVE:   
    case TOKEN_MASTER:  
      if (json_spool_available() )                        // AUX input
      {
        return json_spool_read();
      }
      break;
  }
  
  return 0;
}

/*
 * Spooling function associated with the AUX port
 */
char aux_spool_read(void)                                 // Take something out of the spool
{
    char ch = 0;
    
    if ( aux_spool_in != aux_spool_out )
    {
      ch = aux_spool[aux_spool_out];
      aux_spool_out = (aux_spool_out+1) % sizeof(aux_spool);
    }
    return ch;
}

void aux_spool_put(char ch)                               // Put something into the spool
{
    aux_spool[aux_spool_in] = ch;
    aux_spool_in = (aux_spool_in+1) % sizeof(aux_spool);

    return;
}

/*
 * Spooling functions associated with handling JSON commands
 */
char json_spool_read(void)                                 // Take something out of the spool
{
    char ch = 0;
    
    if ( json_spool_in != json_spool_out )
    {
      ch = json_spool[json_spool_out];
      json_spool_out = (json_spool_out+1) % sizeof(json_spool);
    }
    return ch;
}

void json_spool_put(char ch)                               // Put something into the spool
{
    json_spool[json_spool_in] = ch;
    json_spool_in = (json_spool_in+1) % sizeof(json_spool);

    return;
}


/*-----------------------------------------------------
 * 
 * function: available
 * 
 * brief:    Check for an available character
 * 
 * return:   TRUE if something is waiting
 * 
 *-----------------------------------------------------
 *
 * Each of the available serial ports are polled for
 * a character.
 * 
 * If a token ring is enabled, the get_all() function is 
 * bypassed
 * 
 *-----------------------------------------------------*/
char available_all(void)
{
  if ( Serial.available() || DISPLAY_SERIAL.available() )         // USB is always acive
  {
    return 1;
  }

  switch (json_token)
  {
     case TOKEN_WIFI:
       return esp01_available();
      
     case TOKEN_MASTER:
     case TOKEN_SLAVE:
        return json_spool_available();
  }
  
  return 0;
}

int aux_spool_available(void)
{
  return (aux_spool_in != aux_spool_out);
}

int json_spool_available(void)
{
  return (json_spool_in != json_spool_out);
}
/*-----------------------------------------------------
 * 
 * function: output_to_all
 * 
 * brief:    Send a string to the available serial ports
 * 
 * return:   None
 * 
 *-----------------------------------------------------
 *
 * Send a string to all of the serial devices that are 
 * in use. 
 * 
 *-----------------------------------------------------*/
 void char_to_all(char ch)
 {
  char str_a[2];
  str_a[0] = ch;
  str_a[1] = 0;
  output_to_all(str_a);
  return;
 }
 
 void output_to_all(char *str)
 {
  unsigned int i, j;              // Iteration Counter
  
  Serial.print(str);              // Main USB port
  DISPLAY_SERIAL.print(str);      // Display Serial Port

/*
 * Determine how the output is going to the auxilary port
 */
  if ( esp01_is_present() )       // WiFi Port
  {
    for (i=0; i != esp01_N_CONNECT; i++ )
    {
      esp01_send(str, i);
    }
    return;
  }

  if ( (json_token == TOKEN_WIFI) // Is the AUX port in normal mode?
      || (token_available()))     // Or the token ring is running?
  {
    AUX_SERIAL.print(str);        // No ESP-01, then use just the AUX port
  }

 /*
  * All done, return
  */
  return;
 }

/*-----------------------------------------------------
 * 
 * function: digital_test()
 * 
 * brief:    Exercise the GPIO digital ports
 * 
 * return:   None
 * 
 *-----------------------------------------------------
 *
 * Read in all of the digial ports and report the 
 * results
 * 
 *-----------------------------------------------------*/
void digital_test(void)
{

  int i;
  double       volts;         // Reference Voltage
  
/*
 * Read in the fixed digital inputs
 */
  Serial.print(T("\r\nTime:"));                      Serial.print(micros()/1000000); Serial.print("."); Serial.print(micros()%1000000); Serial.print(T("s"));
  Serial.print(T("\r\nBD Rev:"));                    Serial.print(revision());       
  Serial.print(T("\r\nDIP: 0x"));                    Serial.print(read_DIP(0), HEX); 
  Serial.print(T("\r\nRUN FlipFlop: 0x"));           Serial.print(is_running(), HEX);   
  Serial.print(T("\r\nTemperature: "));              Serial.print(temperature_C());  Serial.print(T("'C "));
  Serial.print(speed_of_sound(temperature_C(), json_rh));  Serial.print(T("mm/us"));
  Serial.print(T("\r\nV_REF: "));                    Serial.print(volts); Serial.print(T(" Volts"));
  Serial.print(T("\r\n"));

/*
 * Read the port pins and report
 */
  i=0;
  while (init_table[i].port != 0xff)
  {
    if ( init_table[i].in_or_out == OUTPUT )
    {
      Serial.print(T("\r\n OUT >> "));
    }
    else
    {
      Serial.print(T("\r\n IN  << "));
    }
    Serial.print(init_table[i].gpio_name); Serial.print(digitalRead(init_table[i].port));
    i++;
  }

 /*
  * Blink the LEDs and exit
  */
   POST_LEDs();
   return;
}


/*----------------------------------------------------------------
 * 
 * function: aquire()
 * 
 * brief: Aquire the data from the counter registers
 * 
 * return: Nothing
 * 
 *----------------------------------------------------------------
 *
 *  This function reads the values from the counters and saves
 *  saves them into the record structure to be reduced later 
 *  on.
 *
 *--------------------------------------------------------------*/
void aquire(void)
 {
/*
 * Pull in the data amd save it in the record array
 */
  if ( DLT(DLT_INFO) )
  {
    Serial.print(T("Aquiring shot:")); Serial.print(this_shot);
  }
  stop_timers();                                    // Stop the counters
  read_timers(&record[this_shot].timer_count[0]);   // Record this count
  record[this_shot].shot_time = FULL_SCALE - in_shot_timer; // Capture the time into the shot
  record[this_shot].face_strike = face_strike;      // Record if it's a face strike
  record[this_shot].sensor_status = is_running();   // Record the sensor status
  record[this_shot].shot_number = shot_number++;    // Record the shot number and increment
  record[this_shot].xphys_mm  = 0;
  record[this_shot].yphys_mm  = 0;                  // Assume we hit a 10.9
  this_shot = (this_shot+1) % SHOT_STRING;          // Prepare for the next shot

/*
 * All done for now
 */
  return;
}

/*----------------------------------------------------------------
 * 
 * function: send_fake_score
 * 
 * brief: Send a fake score to the PC for testing
 * 
 * return: Nothing
 * 
 *----------------------------------------------------------------
 *
 *  This function reads the values from the counters and saves
 *  saves them into the record structure to be reduced later 
 *  on.
 *
 *--------------------------------------------------------------*/
static void send_fake_score(void) 
{ 
  static   shot_record_t shot;
    
  shot.xphys_mm = random(-json_sensor_dia/2.0, json_sensor_dia/2.0);
  shot.yphys_mm = 0;
  shot.shot_number++;
  send_score(&shot);

  return;
} 

/*----------------------------------------------------------------
 * 
 * function: rapid_red()
 *           rapid_green()
 * 
 * brief: Set the RED and GREEN lights
 * 
 * return: Nothing
 * 
 *----------------------------------------------------------------
 *
 *  If MFS2 has enabled the rapid fire lights then allow the 
 *  value to be set
 *
 *--------------------------------------------------------------*/

void rapid_red
(
  unsigned int state          // New state for the RED light
) 
{
  if ( HOLD1(json_multifunction2) == RAPID_RED )
  {
      digitalWrite(DIP_0, state);
  }
  if ( HOLD2(json_multifunction2) == RAPID_RED )
  {
      digitalWrite(DIP_3, state);
  }

  return;
}

void rapid_green
(
  unsigned int state          // New state for the RED light
) 
{
  if ( HOLD1(json_multifunction2) == RAPID_GREEN )
  {
      digitalWrite(DIP_0, state);
  }
  if ( HOLD2(json_multifunction2) == RAPID_GREEN )
  {
      digitalWrite(DIP_3, state);
  }

  return;
}
