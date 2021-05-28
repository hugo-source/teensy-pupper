/*

	Controller
	Controller and planner object

*/

#include "controller.h"


// Calculate the desired foot locations for the next timestep
//
//        Returns
//        -------
//        Numpy array (3, 4)
//            Matrix of new foot locations.
//
void Controller::step_gait( QuadLegs foot_locations, int16_t contact_modes[4] )
{
  LegArray new_location;
  gait_controller.contacts( state->ticks, contact_modes );
  //Serial.print( "contact_modes=" ); Serial.print( contact_modes[0] );
  //Serial.print( ", " ); Serial.print( contact_modes[1] );
  //Serial.print( ", " ); Serial.print( contact_modes[2] );
  //Serial.print( ", " ); Serial.println( contact_modes[3] );
  for ( int leg_index=0; leg_index<4; leg_index++ ) {
    int16_t contact_mode = contact_modes[leg_index];
    //Serial.print( "contact_mode=" ); Serial.println( contact_mode );
    LegArray foot_location; 
    foot_locations.GetLeg( leg_index, foot_location );
    if ( contact_mode == 1 ) {
      //Serial.println( "stance_controller.next_foot_location" );
      //Serial.print( "*^stance_controller ticks=" ); Serial.println( state->ticks );
      //Serial.print( "**leg_index=" ); Serial.println( leg_index );
      stance_controller.next_foot_location( leg_index, new_location ); 
      //Serial.print( "stance=" ); Serial.print( new_location[0], 4 );
      //Serial.print( ", " ); Serial.print( new_location[1], 4 );
      //Serial.print( ", " ); Serial.println( new_location[2], 4 );
    }  else {
      //Serial.println( "gait_controller.subphase_ticks" );
      //Serial.println( "swing_controller.next_foot_location" );
      //Serial.print( "*swing_controller ticks=" ); Serial.println( state->ticks );
      //Serial.print( "**leg_index=" ); Serial.println( leg_index );
      float swing_proportion = gait_controller.subphase_ticks( state-> ticks ) / Config->swing_ticks();
      //Serial.print( "swing_proportion=" ); Serial.println( swing_proportion );
      swing_controller.next_foot_location( swing_proportion, leg_index, new_location );
      //Serial.print( "swing=" ); Serial.print( new_location[0], 4 );
      //Serial.print( ", " ); Serial.print( new_location[1], 4 );
      //Serial.print( ", " ); Serial.println( new_location[2], 4 );

    };
    state->foot_locations.PutLeg( leg_index, new_location ); // update state->foot_locations

    //String label = "foot loc " + String( leg_index );
    //PrintMatrix( (float*) new_location, 1, 3, label );
  };
  //if ( state->debug ) PrintMatrix( (float *)state->foot_locations.Legs, 3, 4, "foot_locations" );
  return;  // foot_locations, contact_modes 
};


// Steps the controller forward one timestep
void Controller::run(  ) { 

  //  ########## Update operating state based on command ######
  if ( cmd->activate_event ) {
    switch ( state->behavior_state ) {
      case DEACTIVATED : state->behavior_state = REST;
      break; 
      case REST : state->behavior_state = DEACTIVATED;
      break;
    }
  } else { 
    if ( cmd->trot_event ) {
      switch( state->behavior_state ) {
        case REST : state->behavior_state = TROT;
        break; 
        case TROT : state->behavior_state = REST;
        break; 
        case HOP  : state->behavior_state = TROT;
        break; 
        case FINISHHOP : state->behavior_state = TROT;
        break;
      }
    } else {
      if ( cmd->hop_event ) {
        switch( state->behavior_state ) {
          case REST : state->behavior_state = HOP;
          break;
          case HOP  : state->behavior_state = FINISHHOP;
          break; 
          case FINISHHOP: state->behavior_state = REST;
          break;
          case TROT : state->behavior_state = HOP;
          break;
        }
      }
    }
  };
  switch ( state->behavior_state ) {
    case TROT :
      if ( state->verbose ) { 
        Serial.println( "TROT" );
        Serial.print( "ticks: " ); Serial.println( uint32_t( state->ticks ) );
      };
      step_gait( state->foot_locations, contact_modes );
      
      if ( state->debug ) PrintMatrix( (float*)state->foot_locations.Legs, 3, 4, "state->foot_locations");
      
      // Apply the desired body rotation
      RotMatrix R;
      EulerAngle euler_a;
      euler_a.X = cmd->roll;
      euler_a.Y = cmd->pitch;
      euler_a.Z = 0;
      Euler2Mat( euler_a, R, ORDER_YXZ );
      //PrintMatrix( (float*)R, 3, 3, "Rotation R");
      Matrix.Multiply( (float*)R, (float*)state->foot_locations.Legs, 3, 3, 4, (float*)rotated_foot_locations.Legs );
      if ( state->debug ) PrintMatrix( (float*)rotated_foot_locations.Legs, 3, 4, "rotated_foot_locations");

      // TODO - incorporate IMU to compensate for bodyroll
      // Construct foot rotation matrix to compensate for body tilt
      //      (roll, pitch, yaw) = quat2euler(state.quat_orientation)
      //      correction_factor = 0.8
      //      max_tilt = 0.4
      //      roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
      //      pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
      //      rmat = euler2mat(roll_compensation, pitch_compensation, 0)

      //      rotated_foot_locations = rmat.T @ rotated_foot_locations

      //      state.joint_angles = self.inverse_kinematics(
      //          rotated_foot_locations, self.config
      //      )
      kinematics.four_legs_inverse_kinematics( rotated_foot_locations, state->joint_angles );
      //PrintMatrix( (float*)state->foot_locations.Legs, 3, 4, "state->foot_locations");
      //PrintMatrix( (float*)state->joint_angles.Legs, 3, 4, "state->joint_angles");
      //PrintMatrix( (float*)a, 3, 4, "state->joint_angles(degrees)");
    break;

    case HOP : ;
      if ( state->verbose ) Serial.println( "HOP" );
      for ( int i=0; i<3; i++ ) {
        for ( int j=0; j<4; j++ ) {
          state->foot_locations.Legs[i][j] = Config->ConfigParams.default_stance[i][j];
          if ( i == 2 ) state->foot_locations.Legs[i][j] = -0.09;
        };
      };
      kinematics.four_legs_inverse_kinematics( state->foot_locations, state->joint_angles );
    break;

    case FINISHHOP : ;
      if ( state->verbose ) Serial.println( "FINISHHOP" );
      for ( int i=0; i<3; i++ ) {
        for ( int j=0; j<4; j++ ) {
          state->foot_locations.Legs[i][j] = Config->ConfigParams.default_stance[i][j];
          if ( i == 2 ) state->foot_locations.Legs[i][j] = -0.22;
        };
      };
      kinematics.four_legs_inverse_kinematics( state->foot_locations, state->joint_angles );
    break;

    case REST : 
       if ( state->verbose ) Serial.println( "REST" ); //
       float yaw_proportion = cmd->yaw_rate / Config->ConfigParams.max_yaw_rate;
       smoothed_yaw += ( Config->ConfigParams.dt * 
                         clipped_first_order_filter( smoothed_yaw,
                         yaw_proportion * -Config->ConfigParams.max_stance_yaw,
                         Config->ConfigParams.max_stance_yaw_rate,
                         Config->ConfigParams.yaw_time_constant ) );
       // Set the foot locations to the default stance plus the standard height
       for ( int i=0; i<3; i++ ) {
         for ( int j=0; j<4; j++ ) {
           state->foot_locations.Legs[i][j] = Config->ConfigParams.default_stance[i][j];
           if ( i == 2 ) state->foot_locations.Legs[i][j] = cmd->height;
         }
       };
       //PrintMatrix( (float*)state->foot_locations.Legs, 3, 4, "state->foot_locations");
       // Apply the desired body rotation
       euler_a.X = cmd->roll;
       euler_a.Y = cmd->pitch;
       euler_a.Z = 0; //smoothed_yaw; TODO: fix this up
       Euler2Mat( euler_a, R, ORDER_YXZ );
       //PrintMatrix( (float*)R, 3, 3, "Rotation R");
       Matrix.Multiply( (float*)R, (float*)state->foot_locations.Legs, 3, 3, 4, (float*)rotated_foot_locations.Legs );
       kinematics.four_legs_inverse_kinematics( rotated_foot_locations, state->joint_angles );
       //PrintMatrix( (float*)rotated_foot_locations.Legs, 3, 4, "rotated_foot_locations");
    break;
  }; // end switch
  
  state->ticks += 1;
  state->pitch = cmd->pitch;
  state->roll = cmd->roll;
  state->height = cmd->height;

};

void Controller::set_pose_to_default( void )
{
  // Set the foot locations to the default stance plus the standard height
  for ( int i=0; i<3; i++ ) {
    for ( int j=0; j<4; j++ ) {
      state->foot_locations.Legs[i][j] = Config->ConfigParams.default_stance[i][j];
      if ( i == 2 ) state->foot_locations.Legs[i][j] = Config->ConfigParams.default_z_ref;
    }
  };
  kinematics.four_legs_inverse_kinematics( state->foot_locations, state->joint_angles );
};
