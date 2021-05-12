/*

	Kinematics
	
*/

#include "kinematics.h"

#include "config.h"
#include <Arduino.h>

void Kinematics::leg_explicit_inverse_kinematics( LegArray r_body_foot, int16_t leg_index, LegArray ret_angles )
{
  float x = r_body_foot[0];
  float y = r_body_foot[1];
  float z = r_body_foot[2];

  //Serial.print( "x " ); Serial.print( x );
  //Serial.print( ", y " ); Serial.print( y );
  //Serial.print( ", z " ); Serial.println( z );

  // Distance from the leg origin to the foot, projected into the y-z plane
  float R_body_foot_yz = sqrt( pow( y, 2 ) + pow( z, 2 ) );

  // Distance from the leg's forward/back point of rotation to the foot
  float R_hip_foot_yz = sqrt( pow( R_body_foot_yz, 2 ) - pow( Config->ConfigParams.ABDUCTION_OFFSET, 2 ) );

  // Interior angle of the right triangle formed in the y-z plane by the leg that is coincident to the ab/adduction axis
  // For feet 2 (front left) and 4 (back left), the abduction offset is positive, for the right feet, the abduction offset is negative.
  float arccos_argument = Config->ConfigParams.ABDUCTION_OFFSETS[leg_index] / R_body_foot_yz;
  arccos_argument = constrain( arccos_argument, -0.99, 0.99 );
  float phi = acos( arccos_argument );

  // Angle of the y-z projection of the hip-to-foot vector, relative to the positive y-axis
  float hip_foot_angle = atan2( z, y );

  // Ab/adduction angle, relative to the positive y-axis
  float abduction_angle = phi + hip_foot_angle;

  // theta: Angle between the tilted negative z-axis and the hip-to-foot vector
  float theta = atan2( -x, R_hip_foot_yz );

  // Distance between the hip and foot
  float R_hip_foot = sqrt( pow( R_hip_foot_yz, 2 ) + pow( x, 2 ) );

  // Angle between the line going from hip to foot and the link L1
  arccos_argument = ( pow( Config->ConfigParams.LEG_L1, 2 ) + pow( R_hip_foot, 2 ) - pow( Config->ConfigParams.LEG_L2, 2) ) / 
                    ( 2 * Config->ConfigParams.LEG_L1 * R_hip_foot );
  arccos_argument = constrain( arccos_argument, -0.99, 0.99 );
  float trident = acos( arccos_argument );

  // Angle of the first link relative to the tilted negative z axis
  float hip_angle = theta + trident;

  // Angle between the leg links L1 and L2
  arccos_argument = ( pow( Config->ConfigParams.LEG_L1, 2 ) + pow( Config->ConfigParams.LEG_L2, 2 ) - pow( R_hip_foot, 2 ) ) / 
                        ( 2 * Config->ConfigParams.LEG_L1 * Config->ConfigParams.LEG_L2 );
  arccos_argument = constrain( arccos_argument, -0.99, 0.99 );
  float beta = acos( arccos_argument );

  // Angle of the second link relative to the tilted negative z axis
  float knee_angle = hip_angle - ( M_PI - beta );

  ret_angles[0] = abduction_angle;
  ret_angles[1] = hip_angle;
  ret_angles[2] = knee_angle;

  //Serial.print( "ret_angles " ); Serial.print( ret_angles[0] );
  //Serial.print( ", " ); Serial.print( ret_angles[1] );
  //Serial.print( ", " ); Serial.println( ret_angles[2] );
  return;
}

//  Find the joint angles for all twelve DOF correspoinding to the given matrix of body-relative foot positions.
//    
//  Parameters
//  ----------
//  r_body_foot : numpy array (3,4)
//      Matrix of the body-frame foot positions. Each column corresponds to a separate foot.
//  config : Config object
//      Object of robot configuration parameters.
//    
//  Returns
//  -------
//  numpy array (3,4)
//      Matrix of corresponding joint angles.
//
void Kinematics::four_legs_inverse_kinematics( QuadLegs r_body_foot, QuadLegs alpha )
{
  for ( int16_t leg_idx=0; leg_idx<4; leg_idx++ ) {
    LegArray leg_body_foot; // leg x, y, z
    LegArray _alpha; // return angles
    r_body_foot.GetLeg( leg_idx, leg_body_foot ); // get a leg
    for ( int i=0; i<3; i++ ) leg_body_foot[i] = leg_body_foot[i] - Config->ConfigParams.LEG_ORIGINS[i][leg_idx];
    leg_explicit_inverse_kinematics( leg_body_foot, leg_idx, _alpha );
    state->joint_angles.PutLeg( leg_idx, _alpha );
    //PrintMatrix( (float *)_alpha, 1, 3, "_alpha" );
  };
  //PrintMatrix( ( float *)alpha.Legs, 3, 4, "alpha" );
  return;
}
