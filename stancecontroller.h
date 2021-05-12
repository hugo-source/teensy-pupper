#ifndef STANCECONTROLLER_H
#define STANCECONTROLLER_H

using namespace std;

#include <Arduino.h>

#include <math.h>
#include <MatrixMath.h>
#include "utilities.h"
#include "command.h"
#include "config.h"
#include "state.h"
#include <Arduino.h>
 

class StanceController 
{
  public:
    Command *cmd;
    Configuration *Config;
    State *state;
 
    void next_foot_location( int16_t leg_index, float incremented_location[3] );
  private:
    void position_delta( int16_t leg_index, float delta_p[3], float delta_R[3][3] );
    
};

 


#endif
