/*
  Auton.hpp

   Created on: March 01, 2021
   Author: 5561
 */

void AutonDriveReset(void);

void AutonDriveMain(double *L_Pct_JoyStickFwdRev,
                    double *L_Pct_JoyStickStrafe,
                    double *L_Pct_JoyStickRotate,
                    double  L_L_X_FieldPos,
                    double  L_L_Y_FieldPos,
                    double  L_Deg_GyroAngleDeg,
                    int     L_int_AutonSelection,
                    bool    L_b_RobotInit);