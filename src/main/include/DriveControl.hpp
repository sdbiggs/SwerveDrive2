/*
  DriveControl.hpp

   Created on: Feb 25, 2020
   Author: 5561
 */

extern double desiredAngle;
extern double rotateDeBounce;
extern double rotateErrorCalc;
extern double rotateErrorIntegral;
extern bool   rotateMode;
extern double V_FWD;
extern double V_STR;
extern double V_RCW;
extern double V_WS[E_RobotCornerSz];
extern double V_WA[E_RobotCornerSz];

void DriveControlMain(double L_JoyStick1Axis1Y,
                      double L_JoyStick1Axis1X,
                      double L_JoyStick1Axis2X,
                      double L_JoyStick1Axis3,
                      bool L_JoyStick1Button1,
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
                      bool *L_TargetFin);



//void DriveControlMain(double c_joyStick.GetRawAxis(1),
//                      double c_joyStick.GetRawAxis(0),
//                      double c_joyStick.GetRawAxis(4),
//                      double c_joyStick.GetRawAxis(3),
//                      double c_joyStick.GetRawButton(3),
//                      double c_joyStick.GetRawButton(4),
//                      double c_joyStick.GetRawButton(5),
//                      double gyro_yawangledegrees,
//                      double gyro_yawanglerad,
//                      double L_WheelAngleFwd,
//                      double L_WheelAngleRev,
//                      double *L_WheelSpeedTarget,
//                      double *L_WheelAngleTarget,
//                      bool   *V_RobotInit);
