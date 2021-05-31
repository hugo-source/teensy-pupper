/*

	Gaits
	
*/

#include "gaits.h"

/*
        """Calculates which part of the gait cycle the robot should be in given the time in ticks.
        
        Parameters
        ----------
        ticks : int
            Number of timesteps since the program started
        gaitparams : GaitParams
            GaitParams object
        
        Returns
        -------
        Int
            The index of the gait phase that the robot should be in.
*/
int GaitController::phase_index( uint32_t ticks )
{
  int phase_time = ticks % Config->phase_length();
  int phase_sum = 0;
  for ( int i=0; i<Config->ConfigParams.num_phases; i++ ) {
    phase_sum += Config->phase_ticks( i );
    if ( phase_time < phase_sum )
      return i;
  };
  return false;
};

/*
  Calculates which feet should be in contact at the given number of ticks
        
        Parameters
        ----------
        ticks : Int
            Number of timesteps since the program started.
        gaitparams : GaitParams
            GaitParams object
        
        Returns
        -------
        numpy array (4,)
            Numpy vector with 0 indicating flight and 1 indicating stance.
*/
void GaitController::contacts( uint32_t ticks, int16_t contact_modes[4] ) 
{
  int idx = phase_index( ticks );
  //Serial.print( "phase_index=" ); Serial.println( idx );
  for ( int i=0; i<4; i++ )
    contact_modes[i] = Config->ConfigParams.contact_phases[i][idx];
  return;
}

/*
        """Calculates the number of ticks (timesteps) since the start of the current phase.

        Parameters
        ----------
        ticks : Int
            Number of timesteps since the program started
        gaitparams : GaitParams
            GaitParams object
        
        Returns
        -------
        Int
            Number of ticks since the start of the current phase.
        """
*/
float GaitController::subphase_ticks( uint32_t ticks )
{
  uint32_t phase_time = ticks % Config->phase_length();
  uint32_t phase_sum = 0;
  float subphase_ticks = 0;
  for ( int i=0; i< Config->ConfigParams.num_phases; i++ ) {
    phase_sum += Config->phase_ticks( i );
    if ( phase_time < phase_sum ) {
      subphase_ticks = phase_time - phase_sum + Config->phase_ticks( i );
      return subphase_ticks;
    };
  };
  return false;
}      
