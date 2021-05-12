/*

	StanceController
	
*/

#include "stancecontroller.h"
/*
        """Calculate the difference between the next desired body location and the current body location
        
        Parameters
        ----------
        z_measured : float
            Z coordinate of the feet relative to the body.
        stance_params : StanceParams
            Stance parameters object.
        movement_reference : MovementReference
            Movement reference object.
        gait_params : GaitParams
            Gait parameters object.

        Returns
        -------
        (Numpy array (3), Numpy array (3, 3))
            (Position increment, rotation matrix increment)
        """

 */
void StanceController::position_delta( int16_t leg_index, float delta_p[3], float delta_R[3][3] )
{
  float z = state->foot_locations.Legs[2][leg_index];
  float v_xy[3];
  v_xy[0] = -cmd->horizontal_velocity[0];
  v_xy[1] = -cmd->horizontal_velocity[1];
  v_xy[2] = 1.0 / Config->ConfigParams.z_time_constant * (state->height - z);
  delta_p[0] = v_xy[0] * Config->ConfigParams.dt;
  delta_p[1] = v_xy[1] * Config->ConfigParams.dt;
  delta_p[2] = v_xy[2] * Config->ConfigParams.dt;
  EulerAngle euler_a;
  euler_a.X = 0;
  euler_a.Y = 0;
  euler_a.Z = -cmd->yaw_rate * Config->ConfigParams.dt;
  Euler2Mat( euler_a, delta_R, ORDER_YXZ );
  return;
};

    //TODO: put current foot location into state
void StanceController::next_foot_location( int16_t leg_index, float incremented_location[3] ) 
{
  float foot_location[3]; 
  state->foot_locations.GetLeg( leg_index, foot_location );
  float delta_p[3];
  float delta_R[3][3];
  //Serial.print( "*ticks=" ); Serial.println( state->ticks );
  //Serial.print( "**leg_index=" ); Serial.println( leg_index );
  //Serial.print( "foot_location=" ); Serial.print( foot_location[0] );
  //Serial.print( ", " ); Serial.print( foot_location[1] );
  //Serial.print( ", " ); Serial.println( foot_location[2] );

  position_delta( leg_index, delta_p, delta_R );
  //Serial.print( "delta_p=" ); Serial.print( delta_p[0] );
  //Serial.print( ", " ); Serial.print( delta_p[1] );
  //Serial.print( ", " ); Serial.println( delta_p[2] );
  
  //PrintMatrix( (float *)delta_R, 3, 3, "delta_R" );
 
  
  Matrix.Multiply( (float*)delta_R, (float*)foot_location, 3, 3, 1, (float*)incremented_location );
  
  //Serial.print( "*incremented_location=" ); Serial.print( incremented_location[0] );
  //Serial.print( ", " ); Serial.print( incremented_location[1] );
  //Serial.print( ", " ); Serial.println( incremented_location[2] );

  incremented_location[0] += delta_p[0];
  incremented_location[1] += delta_p[1];
  incremented_location[2] += delta_p[2];

  //Serial.print( "incremented_location=" ); Serial.print( incremented_location[0] );
  //Serial.print( ", " ); Serial.print( incremented_location[1] );
  //Serial.print( ", " ); Serial.print( incremented_location[2] );

  return;
};
