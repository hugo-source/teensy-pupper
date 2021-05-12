#ifndef CONTROLLER_H
#define CONTROLLER_H

using namespace std;
#include <string.h>
#include <Arduino.h>

#include <math.h>
#include <MatrixMath.h>

#include "state.h"
#include "utilities.h"
#include "kinematics.h"
#include "command.h"
#include "gaits.h"
#include "stancecontroller.h"
#include "swinglegcontroller.h"

class Controller
{

  public:
    State *state;
    Command *cmd;
    Configuration *Config;
    Kinematics kinematics;
    GaitController gait_controller;
    StanceController stance_controller;
    SwingController swing_controller;

    Controller() {};
 
    void run( );
    void set_pose_to_default( void );
        
  private:
    QuadLegs r_body_foot; 
    QuadLegs alpha;
    float smoothed_yaw = 0.0;  // for REST mode only
    int16_t contact_modes[4];
    QuadLegs rotated_foot_locations;
    
    void step_gait( QuadLegs foot_locations, int16_t contact_modes[4] );
};  


#endif
