/*

	JoystickInterface
	
*/

#include "Joystickinterface.h"

JoystickInterface::JoystickInterface(  )
{
  previous_gait_toggle = 0;
  previous_state = REST;
  previous_hop_toggle = 0;
  previous_activate_toggle = 0;

  message_rate = 50;
};

int JoystickInterface::get_command( void )
{
  cmd->trot_event = true;

  // ####### Handle discrete commands ########
  // Check if requesting a state transition to trotting, or from trotting to resting
  int gait_toggle = R1;
  cmd->trot_event = ( gait_toggle == 1 ) and ( previous_gait_toggle == 0 );

  // Check if requesting a state transition to hopping, from trotting or resting
  int hop_toggle = 0; //msg["x"]
  cmd->hop_event = ( hop_toggle == 1 ) and ( previous_hop_toggle == 0 );            
            
  int activate_toggle = L1;
  cmd->activate_event = ( activate_toggle == 1 ) and ( previous_activate_toggle == 0);

  // Update previous values for toggles and state
  previous_gait_toggle = gait_toggle;
  previous_hop_toggle = hop_toggle;
  previous_activate_toggle = activate_toggle;

  //####### Handle continuous commands ########
  float x_vel = ly * Config->ConfigParams.max_x_velocity;
  float y_vel = lx * -Config->ConfigParams.max_y_velocity;
  cmd->horizontal_velocity[0] = x_vel;
  cmd->horizontal_velocity[1] = y_vel;
  cmd->yaw_rate = rx * -Config->ConfigParams.max_yaw_rate;

  //message_rate = msg["message_rate"]
  float message_dt = 1.0 / message_rate;

  float pitch = ry * Config->ConfigParams.max_pitch;
  float deadbanded_pitch = deadband( pitch, Config->ConfigParams.pitch_deadband );
  float pitch_rate = clipped_first_order_filter( state->pitch, deadbanded_pitch,
                Config->ConfigParams.max_pitch_rate,
                Config->ConfigParams.pitch_time_constant );
  cmd->pitch = state->pitch + message_dt * pitch_rate;

  float height_movement = dpady;
  //cmd->height = state->height - message_dt * Config->ConfigParams.z_speed * height_movement;
  cmd->height = height_movement;
            
  float roll_movement = -dpadx;
  cmd->roll = state->roll + message_dt * Config->ConfigParams.roll_speed * roll_movement;
  
  return 1;
};
