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

State state;
Command cmd;
Configuration configuration;

Controller controller;
HardwareInterface hardware_interface;

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
  // read string from serial monitor
  if ( Serial.available() ) {
    comdata = Serial.readStringUntil( '\n' );
    c = comdata.charAt( 0 );
    comdata.remove( 0, 1 );
    printfreeMemory();
    switch( c ) { // command
      case 't' : state.behavior_state = TROT; // trot
      break; 
      case 'e' : state.behavior_state = REST; // rest
      break;
      case 'o' : state.behavior_state = HOP; // hop
      break;
      case 'h' : 
        p = comdata.toFloat();  // height
        cmd.height = p; 
      break;
      case 'r' :
        p = comdata.toFloat();  
        cmd.roll = p; // Get command input here... body roll
      break;
      case 'p' : 
        p = comdata.toFloat();  
        cmd.pitch = p; // body pitch
      break;
      case 'x' : 
        p = comdata.toFloat();  
        cmd.horizontal_velocity[0] = p; // forward/backward motion
      break;
      case 'y' : 
        p = comdata.toFloat();  
        cmd.horizontal_velocity[1] = p; // left/right motion
      break;
    }; 
  };
  
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
    controller.run( );
    // Update the pwm widths going to the servos
    hardware_interface.set_actuator_postions( );
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
 
int freeMemory( void ) {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
