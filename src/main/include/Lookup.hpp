/*
  LookUp.hpp

   Created on: Feb 14, 2018
       Author: sdbig
 */
#ifndef SRC_ROBORIO2018_LOOKUP_HPP_
#define SRC_ROBORIO2018_LOOKUP_HPP_
extern double DesiredSpeed(double L_JoystickAxis);

extern double RampTo(double  L_Final,
                     double  L_Current,
                     double  L_Slope);

extern void DesiredRollerSpeed(double  L_Distance,
                               double  L_Angle,
                               double *L_UpperCmnd,
                               double *L_LowerCmnd);

extern double DesiredRotateSpeed(double L_Error);

extern double DesiredAutoRotateSpeed(double L_Error);

extern double DesiredLowerBeamSpeed(double L_TargetDistance);

extern double DesiredUpperBeamSpeed(double L_TargetDistance);

extern void DesiredAutonLocation(double  L_t_AutonTime,
                                 double *L_L_X_Location,
                                 double *L_L_Y_Location);
#endif /* SRC_ROBORIO2018_LOOKUP_HPP_ */
