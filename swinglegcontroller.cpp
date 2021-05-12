/*

	SwingLegController
	
*/
#include <assert.h>
#include "swinglegcontroller.h"


void SwingController::raibert_touchdown_location( int16_t leg_index, float touchdown_location[3] )
{
  float delta_p_2d[2];
  delta_p_2d[0] = cmd->horizontal_velocity[0] * Config->ConfigParams.alpha * Config->stance_ticks() *
                  Config->ConfigParams.dt;
  delta_p_2d[1] = cmd->horizontal_velocity[1] * Config->ConfigParams.alpha * Config->stance_ticks() *
                  Config->ConfigParams.dt;
                  
  float delta_p[3] = { delta_p_2d[0], delta_p_2d[1], 0 };

  RotMatrix R;
  EulerAngle theta;
  theta.X = 0;
  theta.Y = 0;
  theta.Z = Config->ConfigParams.beta * Config->stance_ticks() * Config->ConfigParams.dt * cmd->yaw_rate;
  Euler2Mat( theta, R, ORDER_YXZ );
  float default_stance[3];
  default_stance[0] = Config->ConfigParams.default_stance[0][leg_index];
  default_stance[1] = Config->ConfigParams.default_stance[1][leg_index];
  default_stance[2] = Config->ConfigParams.default_stance[2][leg_index];
  
  Matrix.Multiply( (float*)R, (float*)default_stance, 3, 3, 1, (float*)touchdown_location );
  touchdown_location[0] += delta_p[0];
  touchdown_location[1] += delta_p[1];
  touchdown_location[2] += delta_p[2];
  
  return;
 };

float SwingController::swing_height( float swing_phase, bool triangular = true )
{
  float swing_height = 0;
  if ( triangular ) {
    if ( swing_phase < 0.5 ) {
      swing_height = swing_phase / 0.5 * Config->ConfigParams.z_clearance;
    } else {
      swing_height = Config->ConfigParams.z_clearance * ( 1 - ( swing_phase - 0.5 ) / 0.5 );
    }
  };    
  return swing_height;
};

void SwingController::next_foot_location( float swing_prop, int16_t leg_index, float foot_location[3] )
{
  assert ( swing_prop >= 0 and swing_prop <= 1 );

  state->foot_locations.GetLeg( leg_index, foot_location );
  float swing_height_ = swing_height( swing_prop );
  float touchdown_location[3];
  raibert_touchdown_location( leg_index, touchdown_location );

  //Serial.print( "raibert_touchdown_location=" ); Serial.print( touchdown_location[0] );
  //Serial.print( ", " ); Serial.print( touchdown_location[1] );
  //Serial.print( ", " ); Serial.println( touchdown_location[2] );

  
  float time_left = Config->ConfigParams.dt * Config->swing_ticks() * ( 1.0 - swing_prop );
  float v[3];
  v[0] = ( touchdown_location[0] - foot_location[0] ) / time_left;
  v[1] = ( touchdown_location[1] - foot_location[1] ) / time_left;
  v[2] = 0;

  float delta_foot_location[3];
  delta_foot_location[0] = v[0] * Config->ConfigParams.dt;
  delta_foot_location[1] = v[1] * Config->ConfigParams.dt;
  delta_foot_location[2] = v[2] + swing_height_ + cmd->height;

  foot_location[0] = foot_location[0] + delta_foot_location[0];
  foot_location[1] = foot_location[1] + delta_foot_location[1];
  foot_location[2] = delta_foot_location[2];

  return;
};
