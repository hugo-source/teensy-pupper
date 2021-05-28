#ifndef JOYSTICK_H
#define JOYSTICK_H

using namespace std;
//#include <string.h>
#include <Arduino.h>


#include <float.h>
#include <stdint.h>
#include <math.h>

#include "utilities.h"
#include "config.h"
#include "state.h"
#include "command.h"
#include "utilities.h"

class JoystickInterface 
{
  public:
    Configuration *Config;
    State *state;
    Command *cmd;
  
    JoystickInterface(  );
    int get_command( void );

    int R1; // gait toggle - REST -> TROT -> REST
    int L1; // activate toggle

    float ly; // horizontal_velocity
    float lx;

    float rx; // yaw_rate
    float ry; // pitch

    float dpadx; // roll movement
    float dpady; // height movement
    
  private:
    bool previous_gait_toggle;
    BehaviorState previous_state;
    bool previous_hop_toggle;
    bool previous_activate_toggle;

    int message_rate;

};  


#endif
