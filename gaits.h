#ifndef GAITS_H
#define GAITS_H

#include <Arduino.h>


#include <float.h>
#include <stdint.h>
#include <math.h>

#include "utilities.h"
#include "config.h"
#include "state.h"


class GaitController 
{

  public:
    Configuration *Config;
    State *state;

    void contacts( int16_t ticks, int16_t contact_modes[4] );
    float subphase_ticks( int ticks );
  
  private:
    int phase_index( int ticks );
};  


#endif
