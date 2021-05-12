#ifndef SWINGLEGCONTROLLER_H
#define SWINGLEGCONTROLLER_H

#include <Arduino.h>

#include <math.h>
#include <MatrixMath.h>
#include "utilities.h"
#include "config.h"
#include "state.h"
#include "command.h"
#include "utilities.h"

class SwingController 
{
  public:
    Configuration *Config;
    State *state;
    Command *cmd;
    void next_foot_location( float swing_prop, int16_t leg_index, float foot_location[3] );

  private:
    float swing_height( float swing_phase, bool triangular = true );
    void raibert_touchdown_location( int16_t leg_index, float touchdown_location[3] );
    
};
 


#endif
