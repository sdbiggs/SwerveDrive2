/*
  Auton.cpp

  Created on: Feb 28, 2021
  Author: 5561

  Changes:
  2021-02-28 -> Bata
 */

#include "Robot.h"

#include "control_pid.hpp"
#include "Encoders.hpp"
#include "DriveControl.hpp"
#include "Lookup.hpp"
#include "Enums.hpp"
#include <math.h>
#include "Gyro.hpp"
#include <frc/smartdashboard/SmartDashboard.h>

double V_t_AutonTime;
double V_L_X_ErrorPrev;
double V_L_Y_ErrorPrev;
double V_L_X_Integral;
double V_L_Y_Integral;
bool   V_b_RecordStartPosition;
double V_L_X_StartPosition;
double V_L_Y_StartPosition;


// using namespace frc;

/******************************************************************************
 * Function:     AutonDriveReset
 *
 * Description:  Reset all applicable Auton variables.
 ******************************************************************************/
void AutonDriveReset(void)
  {
      V_t_AutonTime = 0.0;
      V_L_X_ErrorPrev = 0.0;
      V_L_Y_ErrorPrev = 0.0;
      V_L_X_Integral = 0.0;
      V_L_Y_Integral = 0.0;
      V_b_RecordStartPosition = true;
      V_L_X_StartPosition = 0.0;
      V_L_Y_StartPosition = 0.0;
  }

/******************************************************************************
 * Function:     AutonDriveMain
 *
 * Description:  Main calling function for the auton drive control.
 *               We take in the current field position, then lookup the desired 
 *               position based on the current time.  Based on this "error"
 *               between the current position and the desired position, we
 *               command either the robot to move foward/backward or to strafe
 *               to try and reach the desired position
 ******************************************************************************/
void AutonDriveMain(double *L_Pct_JoyStickFwdRev,
                    double *L_Pct_JoyStickStrafe,
                    double *L_Pct_JoyStickRotate,
                    double  L_L_X_FieldPos,
                    double  L_L_Y_FieldPos,
                    double  L_Deg_GyroAngleDeg,
                    int     L_int_AutonSelection,
                    bool    L_b_RobotInit)
  {
    double L_L_X_Location = 0.0;
    double L_L_Y_Location = 0.0;
    double L_L_FwdRevPosition = 0.0;
    double L_L_Strafe = 0.0;

    if (L_b_RobotInit == false)
      {
        DesiredAutonLocation( V_t_AutonTime,
                             &L_L_X_Location,
                             &L_L_Y_Location);
        
        if (V_b_RecordStartPosition == true)
          {
            V_L_X_StartPosition = L_L_X_Location;
            V_L_Y_StartPosition = L_L_Y_Location;
            V_b_RecordStartPosition = false;
          }

        /* We need to offset the position by the start position since the odometry will 
           start at zero, but the lookup table will not */
        L_L_FwdRevPosition = L_L_X_Location - V_L_X_StartPosition;
        L_L_Strafe = L_L_Y_Location - V_L_Y_StartPosition;

        *L_Pct_JoyStickFwdRev =  Control_PID( L_L_FwdRevPosition,
                                             -L_L_X_FieldPos,
                                             &V_L_X_ErrorPrev,
                                             &V_L_X_Integral,
                                              K_k_AutonX_PID_Gx[E_P_Gx],
                                              K_k_AutonX_PID_Gx[E_I_Gx],
                                              K_k_AutonX_PID_Gx[E_D_Gx],
                                              K_k_AutonX_PID_Gx[E_P_Ul],
                                              K_k_AutonX_PID_Gx[E_P_Ll],
                                              K_k_AutonX_PID_Gx[E_I_Ul],
                                              K_k_AutonX_PID_Gx[E_I_Ll],
                                              K_k_AutonX_PID_Gx[E_D_Ul],
                                              K_k_AutonX_PID_Gx[E_D_Ll],
                                              K_k_AutonX_PID_Gx[E_Max_Ul],
                                              K_k_AutonX_PID_Gx[E_Max_Ll]);

        *L_Pct_JoyStickStrafe =  Control_PID( L_L_Strafe,
                                             -L_L_Y_FieldPos,
                                             &V_L_Y_ErrorPrev,
                                             &V_L_Y_Integral,
                                              K_k_AutonY_PID_Gx[E_P_Gx],
                                              K_k_AutonY_PID_Gx[E_I_Gx],
                                              K_k_AutonY_PID_Gx[E_D_Gx],
                                              K_k_AutonY_PID_Gx[E_P_Ul],
                                              K_k_AutonY_PID_Gx[E_P_Ll],
                                              K_k_AutonY_PID_Gx[E_I_Ul],
                                              K_k_AutonY_PID_Gx[E_I_Ll],
                                              K_k_AutonY_PID_Gx[E_D_Ul],
                                              K_k_AutonY_PID_Gx[E_D_Ll],
                                              K_k_AutonY_PID_Gx[E_Max_Ul],
                                              K_k_AutonY_PID_Gx[E_Max_Ll]);

        V_t_AutonTime += C_ExeTime;
      }
  }