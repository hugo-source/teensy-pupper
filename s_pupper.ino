/*
 * 
 * s_pupper
 * An interpretation in c++ of the Stanford Pupper Python software by Nathan Kau
 * and others.
 * 
 * Hugh Maclaurin 12.May.2021
 * 
 * Version
 * 1.0    2021/05/12
 *  Teensy 4.0
 *  Todo: Add PS4 controller
 * 
 */

using namespace std;
#include <string.h>
#include "utilities.h"

#include "controller.h" 
#include "state.h"
#include "config.h"
#include "command.h"
#include "hardwareinterface.h"
#include "joystickinterface.h"

//DECLARE PINS
//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the IMU
//Radio:
const int ch1Pin = 10; //throttle
const int ch2Pin = 11; //ail
const int ch3Pin = 12; //ele
const int ch4Pin = 14; //rudd
const int ch5Pin = 15; //gear (throttle cut)
const int ch6Pin = 16; //aux1 (free aux channel)
const int ch7Pin = 17;
//const int ch8Pin = 14;
//Radio comm:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_7_pwm, channel_8_pwm, channel_9_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;
unsigned long channel_7_pwm_prev, channel_8_pwm_prev, channel_9_pwm_prev;
//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 1000; //gear
unsigned long channel_6_fs = 1000; //aux1
unsigned long channel_7_fs = 1000; //aux2

float lx;
float ly;
float ry;
float rx;

int Sw1, Sw2, Sw3;

State state;
Command cmd;
Configuration configuration;

Controller controller;
HardwareInterface hardware_interface;
JoystickInterface joystick_interface;

//General stuff
// main loop timing
float dt;
// inner loop timing
const float dt_inner = 0.01;
unsigned long current_time, prev_time;

float last_loop;
const long LOOP_COUNT = 20;

// heartbeat
const int ledPin =  LED_BUILTIN;  // the number of the LED pin
int ledState = LOW;               // ledState used to set the LED
unsigned long previousMillis = 0; // will store last time LED was updated
unsigned long interval;
const long interval250 = 250;     // interval at which to blink (milliseconds)
const long interval500 = 500;     // interval at which to blink (milliseconds)
const long interval1000 = 1000;   // interval at which to blink (milliseconds)

void setup() {
  Serial.begin(115200); // usb serial
  delay( 1000 );
  pinMode( ledPin, OUTPUT );
  digitalWrite( ledPin, HIGH );
  delay( 1500 );  //Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration

  //Initialize radio communication - SELECT ONE 
//  readPWM_setup(ch1Pin, ch2Pin, ch3Pin, ch4Pin, ch5Pin, ch6Pin, ch7Pin, ch8Pin); //uncomment if using 6 channel pwm receiver
  readPWM_setup(ch1Pin, ch2Pin, ch3Pin, ch4Pin, ch5Pin, ch6Pin, ch7Pin); 
  //readPPM_setup(PPM_Pin) //uncomment if using ppm receiver

  //Set radio channels to default (safe) values
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;
  channel_7_pwm = channel_7_fs;

  // Init controller
  controller.state = &state;
  controller.cmd = &cmd; 
  controller.Config = &configuration;
  // Init gait_controller
  controller.gait_controller.Config = &configuration;
  controller.gait_controller.state = &state;
  // Init stance_controller
  controller.stance_controller.Config = &configuration;
  controller.stance_controller.cmd = &cmd;
  controller.stance_controller.state = &state;
  // Init swing_controller
  controller.swing_controller.Config = &configuration;
  controller.swing_controller.state = &state;

  state.debug_str = "pupper";
  state.behavior_state = REST;
  state.verbose = false;
  
  controller.swing_controller.cmd = &cmd;
  // Init kinematics
  controller.kinematics.Config = &configuration;
  controller.kinematics.state = &state;

  hardware_interface.state = &state;
  hardware_interface.Config = &configuration;
  hardware_interface.init_servos();

  joystick_interface.state = &state;
  joystick_interface.Config = &configuration;
  joystick_interface.cmd = &cmd;

  // Default foot positions and angles
  controller.set_pose_to_default();

  current_time = micros(); 
  last_loop = float( micros() / 1000000.0 );
   
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,150,70); //numBlinks, upTime (ms), downTime (ms) -- 3 quick blinks indicates entering main loop!

  Serial.println( "Summary of gait and stance parameters: ");
  Serial.print( "overlap time: " );Serial.println( configuration.ConfigParams.overlap_time );
  Serial.print( "swing time: " ); Serial.println( configuration.ConfigParams.swing_time );
  Serial.print( "z clearance: " ); Serial.println( configuration.ConfigParams.z_clearance );
  Serial.print( "x shift: " ); Serial.println( configuration.ConfigParams.x_shift );
  Serial.println( "Default foot positions:" );
  PrintMatrix( (float*)state.foot_locations.Legs, 3, 4, "state.foot_locations");
  delay( 2500 );
  Serial.println( "starting..." );
}

String comdata = "";

//MAIN LOOP
void loop() {
  char c;
  float p;
  
  //if ( state.ticks > LOOP_COUNT ) while( 1 );
  //Get vehicle commands for next loop iteration
  getCommands(); //pulls current available radio commands
  //Serial.printf( "Channels=%d, %d, %d, %d, %d, %d, %d\n", channel_1_pwm, channel_2_pwm, channel_3_pwm, 
  //                                                    channel_4_pwm, channel_5_pwm, channel_6_pwm, channel_7_pwm );
  //Serial.printf( "Sw1=%d, Sw2=%d, Sw3=%d\n", Sw1, Sw2, Sw3 );
  //Serial.printf( "x=%f, y=%f, pitch=%f, roll=%f (%d, %d, %d)\n", x_vel, y_vel, body_pitch, body_roll, Sw1, Sw2, Sw3 );


//  switch ( Sw3 ) {
//    case 0 : cmd.height = -0.14;
//    break;
//    case 1 : cmd.height = -0.16;
//    break;
//    case 2 : cmd.height = -0.18; 
//    break;
//  };
    

  // read string from serial monitor
//  if ( Serial.available() ) {
//    comdata = Serial.readStringUntil( '\n' );
//    c = comdata.charAt( 0 );
//    comdata.remove( 0, 1 );
//    printfreeMemory();
//    switch( c ) { // command
//      case 't' : state.behavior_state = TROT; // trot
//      break; 
//      case 'e' : state.behavior_state = REST; // rest
//      break;
//      case 'o' : state.behavior_state = HOP; // hop
//      break;
//      case 'h' : 
//        p = comdata.toFloat();  // height
//        cmd.height = p; 
//      break;
//      case 'r' :
//        p = comdata.toFloat();  
//        cmd.roll = p; // Get command input here... body roll
//      break;
//      case 'p' : 
//        p = comdata.toFloat();  
//        cmd.pitch = p; // body pitch
//      break;
//      case 'x' : 
//        p = comdata.toFloat();  
//        cmd.horizontal_velocity[0] = p; // forward/backward motion
//      break;
//      case 'y' : 
//        p = comdata.toFloat();  
//        cmd.horizontal_velocity[1] = p; // left/right motion
//      break;
//    }; 
//  };
 
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0; 

  // Set heartbeat interval
  if ( state.behavior_state == REST ) interval = interval1000; 
  if ( state.behavior_state == TROT ) interval = interval500;
  if ( state.behavior_state == HOP )  interval = interval250;
  if ( state.behavior_state == FINISHHOP )  interval = interval250;

  // Run pupper at ConfigParams.dt intervals
  while ( true ) {
    float now = float( micros() / 1000000.0 );
    if ( ( now - last_loop ) < dt_inner )
      break;
    //Serial.print( "time: " ); Serial.println( ( now - last_loop ) );
    last_loop = float( micros() / 1000000.0 );

    joystick_interface.R1 = Sw1; // gait toggle 
    joystick_interface.ly = ly; // horizontal_velocity
    joystick_interface.lx = lx;

    joystick_interface.rx = rx; // yaw_rate
    joystick_interface.ry = ry; // pitch

    //joystick_interface.dpadx; // roll movement
    if ( Sw3 == 0 ) joystick_interface.dpady = -0.16; // height movement
    if ( Sw3 == 1 ) joystick_interface.dpady = -0.15; // height movement
    if ( Sw3 == 2 ) joystick_interface.dpady = -0.17; // height movement
  
    joystick_interface.get_command();

    controller.run( );
    // Update the pwm widths going to the servos
    hardware_interface.set_actuator_postions( );

    //Serial.printf( "horizontal velocity %f : %f\n", cmd.horizontal_velocity[0], cmd.horizontal_velocity[1] );
    //Serial.printf( "yaw rate %f, pitch %f\n", cmd.yaw_rate, cmd.pitch );
    Serial.printf( "height %f\n", cmd.height );

  }; // end while

  // Pupper heartbeat
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    ( ledState == LOW ) ? ledState = HIGH : ledState = LOW;
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  };

  //Regulate main loop rate
  loopRate( 2000 ); //  2000 Hz, 
}

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters scattered around this code.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  };
};

void setupBlink(int numBlinks,int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  };
};

int GetSw( unsigned long raw_value )
{
  if ( raw_value < 1300 )
    return 0;
  else
    if ( raw_value < 1700 )
      return 1;
    else
      return 2;
  
}
void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop. The radio commands are retrieved from a function in the readPWM
   * file separate from this one which is running a bunch of interrupts to continuously update the radio readings. 
   * The raw radio commands are being filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

  //radio_command = 1;
  channel_1_pwm = getRadioPWM(1);
  channel_2_pwm = getRadioPWM(2);
  channel_3_pwm = getRadioPWM(3);
  channel_4_pwm = getRadioPWM(4);
  channel_5_pwm = getRadioPWM(5);
  channel_6_pwm = getRadioPWM(6);
  channel_7_pwm = getRadioPWM(7);
//  channel_8_pwm = getRadioPWM(8);
//  channel_9_pwm = getRadioPWM(9);
  
  //Low-pass the critical commands and update previous values
  float b = 0.2;
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
  // Switches
  Sw1 = GetSw( channel_5_pwm );
  Sw2 = GetSw( channel_6_pwm );
  Sw3 = GetSw( channel_7_pwm );
  channel_1_pwm -= 70;
  // Channel limits
  ly = (channel_1_pwm - 1000.0)/1000.0; //between 0 and 1
  lx = (channel_2_pwm - 1500.0)/500.0; //between -1 and 1

  ry = (channel_3_pwm - 1500.0)/500.0; // pitch between -1 and 1
  rx = (channel_4_pwm - 1500.0)/500.0; // yaw_rate between -1 and 1

  //Constrain within normalized bounds
  lx = trunc( lx * 100 ) / 100;
  ly = trunc( ly * 100 ) / 100;
  ry = trunc( ry * 100 ) / 100;
  rx = trunc( rx * 100 ) / 100;
}

// Keep track of dynamic memory
void printfreeMemory( void )
{
  Serial.print( "Free memory=" ); Serial.println( freeMemory() );  
}

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
 
unsigned long freeMemory( void ) {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
