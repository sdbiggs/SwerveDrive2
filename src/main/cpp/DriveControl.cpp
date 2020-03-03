/*
  DriveControl.cpp

  Created on: Feb 25, 2020
  Author: 5561
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


double desiredAngle;
double rotateDeBounce;
double rotateErrorCalc;
double rotateErrorIntegral;
bool   rotateMode;
bool   autoBeamLock;
double V_FWD;
double V_STR;
double V_RCW;
double V_WS[E_RobotCornerSz];
double V_WA[E_RobotCornerSz];



/******************************************************************************
 * Function:     DriveControlMain
 *
 * Description:  Main calling function for the drive control.
 ******************************************************************************/
void DriveControlMain(double L_JoyStick1Axis1Y,
                      double L_JoyStick1Axis1X,
                      double L_JoyStick1Axis2X,
                      double L_JoyStick1Axis3,
                      double L_JoyStick1Button1,
                      double L_JoyStick1Button3,
                      double L_JoyStick1Button4,
                      double L_JoyStick1Button5,
                      double L_GyroAngleDegrees,
                      double L_GyroAngleRadians,
                      double L_VisionAngleDeg,
                      double *L_WheelAngleFwd,
                      double *L_WheelAngleRev,
                      double *L_WheelSpeedTarget,
                      double *L_WheelAngleTarget,
                      bool   *L_RobotInit,
                      T_AutoTargetStates L_AutoTargetState,
                      double L_DesiredAngle)
  {
  int    L_Index;
  double L_temp;
  double L_A;
  double L_B;
  double L_C;
  double L_D;
  double L_Gain;
  double L_Max;
  double L_WA_FWD;
  double L_WA_FWD_Delta;
  double L_WA_REV;
  double L_WA_REV_Delta;
  double L_WA[E_RobotCornerSz];
  double L_WS[E_RobotCornerSz];
  double L_RotateErrorCalc;
  bool   L_Init = *L_RobotInit;

  /* Let's start by zeroing the desired angle and speed */
  if (L_Init == true)
    {
    L_Init = false;
    for (L_Index = E_FrontLeft;
         L_Index < E_RobotCornerSz;
         L_Index = T_RobotCorner(int(L_Index) + 1))
      {
      V_WheelAngleArb[L_Index] = L_WheelAngleFwd[L_Index]; // We do this for initialization in order to allow the PID control to control to the correct forward angle at startup
       if (fabs(V_WheelAngleArb[L_Index]) > K_InitAngle)
         {
         L_Init = true;
         }
      }
    }

  /* Check to see if we are in initialization.
   * If not, we can do normal control. */
  if (L_Init == false)
    {
    /* Let's place a deadband around the joystick readings */
    V_FWD = L_JoyStick1Axis1Y * -1;
    V_STR = L_JoyStick1Axis1X;
    V_RCW = L_JoyStick1Axis2X;

   //turning rotatemode on/off & setting desired angle
    // if (L_AutoTargetState != E_NotActive)
    //   {
    //   desiredAngle = L_DesiredAngle;
    //   }
    if ((fabs(L_JoyStick1Axis1Y) > 0) ||
        (fabs(L_JoyStick1Axis1X) > 0)  ||
        (fabs(L_JoyStick1Axis2X) > 0))
    {
      autoBeamLock = false;
      rotateMode = false;
    }
    else if(L_JoyStick1Button1 || autoBeamLock == true)
    {
      /* Auto targeting */
      autoBeamLock = true;
      desiredAngle = 0; // This is due to the offset of the camera
    }
    else if (L_JoyStick1Button4)
      {
      rotateMode = true;
      desiredAngle = 90;
      }
    else if (L_JoyStick1Button3)
      {
      rotateMode = true;
      desiredAngle = 0;
      }
    else if (L_JoyStick1Button5)
      {
      rotateMode = true;
      desiredAngle = 67.5;
      }
    if (L_JoyStick1Button5 && gyro_yawangledegrees == 67.5) {
      GyroZero();
    }

  
    if (rotateMode == true)
      {
      L_RotateErrorCalc = desiredAngle - L_GyroAngleDegrees;
      }
    else if(autoBeamLock == true)
      {
      L_RotateErrorCalc = desiredAngle - L_VisionAngleDeg;
      }
    else
      {
      L_RotateErrorCalc = 0;
      }
    
    frc::SmartDashboard::PutNumber("L_RotateErrorCalc", L_RotateErrorCalc);

    if ((rotateMode == true   && fabs(L_RotateErrorCalc) <= 5 && rotateDeBounce <= 0.02) || 
        (autoBeamLock == true && fabs(L_RotateErrorCalc) <= 5 && rotateDeBounce <= 0.02))
      {
      // rotateMode = true;
      // autoBeamLock = true;
      rotateDeBounce += 0.01;
      }
    else if ((rotateMode == true   && fabs(L_RotateErrorCalc) <= 5 && rotateDeBounce >= 0.02) ||
             (autoBeamLock == true && fabs(L_RotateErrorCalc) <= 5 && rotateDeBounce >= 0.02))
      {
      rotateMode = false;
      autoBeamLock = false;
      rotateDeBounce = 0;
      }

    if ((rotateMode == true) || 
        (autoBeamLock == true))
      {
      V_RCW = Control_PID(desiredAngle,
                          L_GyroAngleDegrees,
                          &rotateErrorCalc,
                          &rotateErrorIntegral,
                          K_RobotRotationPID_Gx[E_P_Gx],
                          K_RobotRotationPID_Gx[E_I_Gx],
                          K_RobotRotationPID_Gx[E_D_Gx],
                          K_RobotRotationPID_Gx[E_P_Ul],
                          K_RobotRotationPID_Gx[E_P_Ll],
                          K_RobotRotationPID_Gx[E_I_Ul],
                          K_RobotRotationPID_Gx[E_I_Ll],
                          K_RobotRotationPID_Gx[E_D_Ul],
                          K_RobotRotationPID_Gx[E_D_Ll],
                          K_RobotRotationPID_Gx[E_Max_Ul],
                          K_RobotRotationPID_Gx[E_Max_Ll]);
      
      }

  
    L_temp = V_FWD * cos(L_GyroAngleRadians) + V_STR * sin(L_GyroAngleRadians);
    V_STR = -V_FWD * sin(L_GyroAngleRadians) + V_STR * cos(L_GyroAngleRadians);
    V_FWD = L_temp;

    //Ws1: fr, Ws2: fl, ws3: rl, ws4: rr
    L_A = V_STR - V_RCW * (C_L/C_R);
    L_B = V_STR + V_RCW * (C_L/C_R);
    L_C = V_FWD - V_RCW * (C_W/C_R);
    L_D = V_FWD + V_RCW * (C_W/C_R);

    L_WS[E_FrontRight] = pow((L_B * L_B + L_C * L_C), 0.5);
    L_WS[E_FrontLeft]  = pow((L_B * L_B + L_D * L_D), 0.5);
    L_WS[E_RearLeft]   = pow((L_A * L_A + L_D * L_D), 0.5);
    L_WS[E_RearRight]  = pow((L_A * L_A + L_C * L_C), 0.5);

    L_WA[E_FrontRight] = atan2(L_B, L_C) *180/C_PI;
    L_WA[E_FrontLeft]  = atan2(L_B, L_D) *180/C_PI;
    L_WA[E_RearLeft]   = atan2(L_A, L_D) *180/C_PI;
    L_WA[E_RearRight]  = atan2(L_A, L_C) *180/C_PI;

    L_Max = L_WS[E_FrontRight];

    if (L_WS[E_FrontLeft] > L_Max)
      {
      L_Max = L_WS[E_FrontLeft];
      }
    if (L_WS[E_RearLeft] > L_Max)
      {
      L_Max = L_WS[E_RearLeft];
      }
    if (L_WS[E_RearRight] > L_Max)
      {
      L_Max = L_WS[E_RearRight];
      }

    if (L_Max > 1)
      {
      L_WS[E_FrontRight] /= L_Max;
      L_WS[E_FrontLeft]  /= L_Max;
      L_WS[E_RearLeft]   /= L_Max;
      L_WS[E_RearRight]  /= L_Max;
      }

    L_Gain = K_RotateDebounceThreshold;

    if (L_JoyStick1Axis3 > L_Gain)
      {
      L_Gain = L_JoyStick1Axis3;
      }
      else if ((rotateMode == true) ||
               (autoBeamLock == true))
      {
      L_Gain = K_AutoRotateGx;
      }

    L_WS[E_FrontRight] *= (K_WheelMaxSpeed * L_Gain);
    L_WS[E_FrontLeft]  *= (K_WheelMaxSpeed * L_Gain);
    L_WS[E_RearLeft]   *= (K_WheelMaxSpeed * L_Gain);
    L_WS[E_RearRight]  *= (K_WheelMaxSpeed * L_Gain);

    for (L_Index = E_FrontLeft;
        L_Index < E_RobotCornerSz;
        L_Index = T_RobotCorner(int(L_Index) + 1))
      {
      L_WA_FWD = DtrmnEncoderRelativeToCmnd(L_WA[L_Index],
                                            L_WheelAngleFwd[L_Index]);

      L_WA_FWD_Delta = fabs(L_WA[L_Index] - L_WA_FWD);

      L_WA_REV = DtrmnEncoderRelativeToCmnd(L_WA[L_Index],
                                            L_WheelAngleRev[L_Index]);

      L_WA_REV_Delta = fabs(L_WA[L_Index] - L_WA_REV);

      if (L_WA_FWD_Delta <= L_WA_REV_Delta)
        {
          V_WheelAngleArb[L_Index] = L_WA_FWD;
        }
      else
        {
          V_WheelAngleArb[L_Index] = L_WA_REV;
          L_WS[L_Index] *= (-1); // Need to flip sign of drive wheel to account for reverse direction
        }
      }
    }

  /* Output the wheel speed and angle targets along with init state: */
    for (L_Index = E_FrontLeft;
        L_Index < E_RobotCornerSz;
        L_Index = T_RobotCorner(int(L_Index) + 1))
      {
      L_WheelSpeedTarget[L_Index] = L_WS[L_Index];
      L_WheelAngleTarget[L_Index] = L_WA[L_Index];
      }
    *L_RobotInit = L_Init;
  }
