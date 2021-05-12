#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>
#include "utilities.h"
#include "config.h"
#include "state.h"

class Kinematics 
{

  public:
    Configuration *Config;
    State *state;
    
    void leg_explicit_inverse_kinematics( LegArray, int16_t leg_index, LegArray );
    void four_legs_inverse_kinematics( QuadLegs r_body_foot, QuadLegs alpha );
  private:

};  


#endif
