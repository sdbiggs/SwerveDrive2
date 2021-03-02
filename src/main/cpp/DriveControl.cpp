/*
  DriveControl.cpp

  Created on: Feb 25, 2020
  Author: 5561

  Changes:
  2021-02-25 -> Updates to help the robot drive straight
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
bool   V_b_DriveStraight;
bool   V_AutoRotateComplete;
double V_FWD;
double V_STR;
double V_RCW;
double V_WS[E_RobotCornerSz];
double V_WA[E_RobotCornerSz];
double V_Deg_DesiredAngPrev = 0;



/******************************************************************************
 * Function:     DriveControlMain
 *
 * Description:  Main calling function for the drive control.
 ******************************************************************************/
void DriveControlMain(double              L_JoyStick1Axis1Y,
                      double              L_JoyStick1Axis1X,
                      double              L_JoyStick1Axis2X,
                      double              L_JoyStick1Axis3,
                      bool                L_JoyStick1Button1,
                      double              L_JoyStick1Button3,
                      double              L_JoyStick1Button4,
                      double              L_JoyStick1Button5,
                      double              L_GyroAngleDegrees,
                      double              L_GyroAngleRadians,
                      double              L_VisionAngleDeg,
                      double             *L_WheelAngleFwd,
                      double             *L_WheelAngleRev,
                      double             *L_WheelSpeedTarget,
                      double             *L_WheelAngleTarget,
                      bool               *L_RobotInit,
                      bool               *L_TargetFin)
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
      V_Deg_DesiredAngPrev = L_GyroAngleDegrees;
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
    if ((fabs(L_JoyStick1Axis1Y) > 0) ||
        (fabs(L_JoyStick1Axis1X) > 0) ||
        (fabs(L_JoyStick1Axis2X) > 0))
      {
      // Abort out of auto rotate and/or auto target if the driver moves the joysticks
      autoBeamLock = false;
      V_AutoRotateComplete = false;
      rotateMode = false;

      if (fabs(L_JoyStick1Axis2X) > 0)
        {
        // Ok, the driver is attempting to rotate the robot 
        desiredAngle = L_GyroAngleDegrees;
        V_b_DriveStraight   = false;
        }
      else
        {
        // Ok, the driver is attempting to not rotate the robot (i.e. drive straight and/or strafe)
        // Set the desired angle of the robot equal to the previously measured/commanded angle and enable rotate mode
        desiredAngle = V_Deg_DesiredAngPrev;
        V_b_DriveStraight   = true;
        }  
      }
    else if(L_JoyStick1Button1 || autoBeamLock == true)
      {
      /* Auto targeting */
      V_b_DriveStraight = false;
      autoBeamLock = true;
      desiredAngle = K_TargetVisionAngle; // This is due to the offset of the camera
      }
    else if (L_JoyStick1Button4)
      {
      V_b_DriveStraight = false;
      rotateMode = true;
      desiredAngle = 90;
      }
    else if (L_JoyStick1Button3)
      {
      V_b_DriveStraight = false;
      rotateMode = true;
      desiredAngle = 0;
      }
    else if (L_JoyStick1Button5)
      {
      V_b_DriveStraight = false;
      rotateMode = true;
      desiredAngle = 67.5;
      }
    else if ((fabs(L_JoyStick1Axis1Y) == 0) &&
             (fabs(L_JoyStick1Axis1X) == 0) &&
             (fabs(L_JoyStick1Axis2X) == 0))
      {
      // No driver input
      desiredAngle      = L_GyroAngleDegrees;
      V_b_DriveStraight = false;
      }
      
    if ((rotateMode == true) ||
        (V_b_DriveStraight == true))
      {
      // Use gyro as target when in auto rotate or drive straight mode
      L_RotateErrorCalc = desiredAngle - L_GyroAngleDegrees;
      }
    else if(autoBeamLock == true)
      {
      // Use chameleon vison as target when in auto beam lock
      L_RotateErrorCalc = desiredAngle - L_VisionAngleDeg;
      V_AutoRotateComplete = false;
      }
    else
      {
      L_RotateErrorCalc = 0;
      }
    
    // frc::SmartDashboard::PutNumber("L_RotateErrorCalc", L_RotateErrorCalc);

    if ((V_b_DriveStraight == true && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce <= K_RotateDebounceTime) ||
        (rotateMode        == true && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce <= K_RotateDebounceTime) || 
        (autoBeamLock      == true && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce <= K_RotateDebounceTime))
      {
      V_AutoRotateComplete = false;      // rotateMode = true;
      // autoBeamLock = true;
      rotateDeBounce += C_ExeTime;
      }
    else if ((V_b_DriveStraight == true && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce >= K_RotateDebounceTime) ||
             (rotateMode        == true && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce >= K_RotateDebounceTime) ||
             (autoBeamLock      == true && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce >= K_RotateDebounceTime))
      {
      rotateMode = false;
      autoBeamLock = false;
      rotateDeBounce = 0;
      V_AutoRotateComplete = true;
      *L_TargetFin = true;
      }

    if (rotateMode == true)
      {
      V_RCW = DesiredRotateSpeed(L_RotateErrorCalc);
      }
    else if (V_b_DriveStraight == true)
      {
      V_RCW = DesiredAutoRotateSpeed(L_RotateErrorCalc);
      }
    else if (autoBeamLock == true)
      {
      V_RCW = -DesiredRotateSpeed(L_RotateErrorCalc);
      }
  
    L_temp =  V_FWD * cos(L_GyroAngleRadians) + V_STR * sin(L_GyroAngleRadians);
    V_STR  = -V_FWD * sin(L_GyroAngleRadians) + V_STR * cos(L_GyroAngleRadians);
    V_FWD  =  L_temp;

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
    else if ((rotateMode   == true) ||
             (autoBeamLock == true))
      {
      L_Gain = K_AutoRotateGx;
      }

    if (L_Gain >= K_MaxGain)
      {
      L_Gain = K_MaxGain;
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

    V_Deg_DesiredAngPrev = desiredAngle;

    frc::SmartDashboard::PutBoolean("AutoRotate Complete",V_AutoRotateComplete);
  }
