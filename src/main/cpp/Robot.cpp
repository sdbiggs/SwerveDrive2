/*
 * Team 5561 2020 Code
 *
 * This code runs the 2020 robot which is capable of the following:
 * - Swerve Drive (beta 02/10/2020)
 * - Shooting balls (beta 02/15/2020)
 *
 * */

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>

#include "Encoders.hpp"
#include "Enums.hpp"
#include "control_pid.hpp"
#include "ColorSensor.hpp"
#include "Gyro.hpp"
#include "Lookup.hpp"
#include "WheelOfFortune.hpp"
#include "vision.hpp"

T_WheelOfFortuneColor V_ColorWheelColor;

double V_FWD;
double V_STR;
double V_RCW;
double V_WS[E_RobotCornerSz];
double V_WA[E_RobotCornerSz];

double V_WheelAngleCmnd[E_RobotCornerSz];
double V_WheelAngleError[E_RobotCornerSz];
double V_WheelAngleIntegral[E_RobotCornerSz];
double V_WheelSpeedCmnd[E_RobotCornerSz];
double V_WheelSpeedError[E_RobotCornerSz];
double V_WheelSpeedIntergral[E_RobotCornerSz];

double V_ShooterSpeedDesired[E_RoboShooter];

bool   V_RobotInit;

frc::LiveWindow *lw = frc::LiveWindow::GetInstance();
std::shared_ptr<NetworkTable> vision0;
std::shared_ptr<NetworkTable> vision1;
std::shared_ptr<NetworkTable> lidar;
std::shared_ptr<NetworkTable> ledLight;

nt::NetworkTableInstance inst;
nt::NetworkTableEntry driverMode0;
nt::NetworkTableEntry targetYaw0;
nt::NetworkTableEntry targetPitch0;
nt::NetworkTableEntry targetPose0;
nt::NetworkTableEntry latency0;
nt::NetworkTableEntry driverMode1;
nt::NetworkTableEntry targetYaw1;
nt::NetworkTableEntry targetPitch1;
nt::NetworkTableEntry targetPose1;
nt::NetworkTableEntry latency1;
nt::NetworkTableEntry lidarDistance;
nt::NetworkTableEntry ledControl;

double       distanceTarget;
double       distanceBall;
double       distanceFromTargetCenter;
double       distanceFromBallCenter;
double       desiredVisionAngle0;
double       desiredVisionDistance0;
bool         activeVisionAngle0;
bool         activeVisionDistance0;
bool         visionRequest;
bool         visionStart1;
bool         visionStart2;

char         autonChoose;

double desiredAngle;
double rotateDeBounce;
double rotateErrorCalc;
double rotateErrorIntegral;
bool   rotateMode;
double SpeedRecommend;

double PDP_Current_UpperShooter = 0;
double PDP_Current_LowerShooter = 0;
double PDP_Current_UpperShooter_last = 0;
double PDP_Current_LowerShooter_last = 0;

double BallsShot = 0;


/******************************************************************************
 * Function:     RobotInit
 *
 * Description:  Called during initialization of the robot.
 ******************************************************************************/
void Robot::RobotInit() {
//  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
//  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
//  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    m_frontLeftSteerMotor.RestoreFactoryDefaults();
    m_frontLeftDriveMotor.RestoreFactoryDefaults();
    m_frontRightSteerMotor.RestoreFactoryDefaults();
    m_frontRightDriveMotor.RestoreFactoryDefaults();
    m_rearLeftSteerMotor.RestoreFactoryDefaults();
    m_rearLeftDriveMotor.RestoreFactoryDefaults();
    m_rearRightSteerMotor.RestoreFactoryDefaults();
    m_rearRightDriveMotor.RestoreFactoryDefaults();
    m_topShooterMotor.RestoreFactoryDefaults();
    m_bottomShooterMotor.RestoreFactoryDefaults();

    V_RobotInit = false;

    GyroRobotInit();

    compressor.SetClosedLoopControl(true);
    inst = nt::NetworkTableInstance::Create();
    inst.StartClient("10.55.61.24");
    inst.StartDSClient();

    vision0  = inst.GetTable("chameleon-vision/goal");
    vision1  = inst.GetTable("chameleon-vision/ball");
    lidar    = inst.GetTable("lidar");
    ledLight = inst.GetTable("ledLight");


    driverMode0           = vision0->GetEntry("driverMode");
    targetPitch0          = vision0->GetEntry("targetPitch");
    targetYaw0            = vision0->GetEntry("targetYaw");
    targetPose0           = vision0->GetEntry("targetpose");
    latency0              = vision0->GetEntry("latency");

    driverMode1           = vision1->GetEntry("driverMode");
    targetPitch1          = vision1->GetEntry("targetPitch");
    targetYaw1            = vision1->GetEntry("targetYaw");
    targetPose1           = vision1->GetEntry("targetpose");
    latency1              = vision1->GetEntry("latency");

    ledControl            = ledLight->GetEntry("ledControl");
    lidarDistance         = lidar->GetEntry("lidarDistance");

    frc::SmartDashboard::PutNumber("Upper_P_Gx", 0);
    frc::SmartDashboard::PutNumber("Upper_I_Gx", 0);
    frc::SmartDashboard::PutNumber("Upper_D_Gx", 0);
    frc::SmartDashboard::PutNumber("Upper_I_Zone", 0);
    frc::SmartDashboard::PutNumber("Upper_FF", 0);
    frc::SmartDashboard::PutNumber("Upper_Max_Limit", 0);
    frc::SmartDashboard::PutNumber("Upper_Min_Limit", 0);

    frc::SmartDashboard::PutNumber("Lower_P_Gx", 0);
    frc::SmartDashboard::PutNumber("Lower_I_Gx", 0);
    frc::SmartDashboard::PutNumber("Lower_D_Gx", 0);
    frc::SmartDashboard::PutNumber("Lower_I_Zone", 0);
    frc::SmartDashboard::PutNumber("Lower_FF", 0);
    frc::SmartDashboard::PutNumber("Lower_Max_Limit", 0);
    frc::SmartDashboard::PutNumber("Lower_Min_Limit", 0);

    frc::SmartDashboard::PutNumber("Speed Desired Top", 0);
    frc::SmartDashboard::PutNumber("Speed Desired Bottom", 0);

    V_ShooterSpeedDesired[E_TopShooter] = 0;
    V_ShooterSpeedDesired[E_BottomShooter] = 0;
    V_ShooterSpeedCurr[E_TopShooter] = 0;
    V_ShooterSpeedCurr[E_BottomShooter] = 0;

    m_topShooterpid.SetP(Upper_P_Gx);
    m_topShooterpid.SetI(Upper_I_Gx);
    m_topShooterpid.SetD(Upper_D_Gx);
    m_topShooterpid.SetIZone(Upper_I_Zone);
    m_topShooterpid.SetFF(Upper_FF);
    m_topShooterpid.SetOutputRange(Upper_Min,Upper_Max);

    m_bottomShooterpid.SetP(Lower_P_Gx);
    m_bottomShooterpid.SetI(Lower_I_Gx);
    m_bottomShooterpid.SetD(Lower_D_Gx);
    m_bottomShooterpid.SetIZone(Lower_I_Zone);
    m_bottomShooterpid.SetFF(Lower_FF);
    m_bottomShooterpid.SetOutputRange(Lower_Min,Lower_Max);

    m_liftpid.SetP(kP);
    m_liftpid.SetI(kI);
    m_liftpid.SetD(kD);
    m_liftpid.SetIZone(kIz);
    m_liftpid.SetFF(kFF);
    m_liftpid.SetOutputRange(kMinOutput, kMaxOutput);

    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("Desired Level", 0);
}


/******************************************************************************
 * Function:     RobotPeriodic
 *
 * Description:  Function called periodically (not defined well as to what
 *               "periodically" means).
 ******************************************************************************/
void Robot::RobotPeriodic()
{
  frc::SmartDashboard::PutNumber("Postion", m_encoderLift.GetPosition());
}


/******************************************************************************
 * Function:     AutonomousInit
 *S
 * Description:  Function called at init while in autonomous.  This is where we
 *               should zero out anything that we need to before autonomous mode.
 ******************************************************************************/
void Robot::AutonomousInit()
  {
//  m_autoSelected = m_chooser.GetSelected();
//  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
//  //     kAutoNameDefault);
//  std::cout << "Auto selected: " << m_autoSelected << std::endl;
//
//  if (m_autoSelected == kAutoNameCustom) {
//    // Custom Auto goes here
//  } else {
//    // Default Auto goes here
//  }
  }


/******************************************************************************
 * Function:     AutonomousPeriodic
 *
 * Description:  Function called periodically in autonomous.  This is where we
 *               should place our primary autonomous control code.
 ******************************************************************************/
void Robot::AutonomousPeriodic()
  {
//  if (m_autoSelected == kAutoNameCustom) {
//    // Custom Auto goes here
//  } else {
//    // Default Auto goes here
//  }
  }


/******************************************************************************
 * Function:     TeleopInit
 *
 * Description:  Function called when starting out in teleop mode.
 *               We should zero out all of our global varibles.
 ******************************************************************************/
void Robot::TeleopInit()
  {
  int index;

  V_RobotInit = true;
  m_frontLeftSteerMotor.RestoreFactoryDefaults();
  m_frontLeftDriveMotor.RestoreFactoryDefaults();
  m_frontRightSteerMotor.RestoreFactoryDefaults();
  m_frontRightDriveMotor.RestoreFactoryDefaults();
  m_rearLeftSteerMotor.RestoreFactoryDefaults();
  m_rearLeftDriveMotor.RestoreFactoryDefaults();
  m_rearRightSteerMotor.RestoreFactoryDefaults();
  m_rearRightDriveMotor.RestoreFactoryDefaults();

  m_frontLeftSteerMotor.SetSmartCurrentLimit(25);
  m_frontRightSteerMotor.SetSmartCurrentLimit(25);
  m_rearLeftSteerMotor.SetSmartCurrentLimit(25);
  m_frontLeftSteerMotor.SetSmartCurrentLimit(25);

  for (index = E_FrontLeft;
       index < E_RobotCornerSz;
       index = T_RobotCorner(int(index) + 1))
      {
        V_WS[index] = 0;
        V_WA[index] = 0;
        V_WheelRelativeAngleRawOffset[index] = 0;
        V_WheelAngleFwd[index] = 0;
        V_WheelAnglePrev[index] = 0;
        V_WheelAngleLoop[index] = 0;
        V_WheelAngleRaw[index] = 0;
        V_WheelAngleError[index] = 0;
        V_WheelAngleIntegral[index] = 0;
        V_WheelVelocity[index] = 0;
        V_WheelSpeedError[index] = 0;
        V_WheelSpeedIntergral[index] = 0;
        V_WheelAngleArb[index] = 0;
      }
      V_STR = 0;
      V_FWD = 0;
      V_RCW = 0;
      gyro_yawangledegrees = 0;
      gyro_yawanglerad = 0;

  BallsShot = 0;

  GyroTeleInit();
}


/******************************************************************************
 * Function:     TeleopPeriodic
 *
 * Description:  Primary function called when in teleop mode.
 ******************************************************************************/
void Robot::TeleopPeriodic()
  {
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
  T_RobotCorner index;
  double L_FortuneMotor;
  T_WheelOfFortuneColor L_Color;

  L_Color = ColorSensor(false);
  Gyro();
  L_FortuneMotor = WheelOfFortune (L_Color,
                                   c_joyStick2.GetRawButton(2),
                                   c_joyStick2.GetRawButton(4),
                                   c_joyStick2.GetRawButton(3));

  Read_Encoders(V_RobotInit,
                a_encoderFrontLeftSteer.GetVoltage(),
                a_encoderFrontRightSteer.GetVoltage(),
                a_encoderRearLeftSteer.GetVoltage(),
                a_encoderRearRightSteer.GetVoltage(),
                m_encoderFrontLeftSteer,
                m_encoderFrontRightSteer,
                m_encoderRearLeftSteer,
                m_encoderRearRightSteer,
                m_encoderFrontLeftDrive,
                m_encoderFrontRightDrive,
                m_encoderRearLeftDrive,
                m_encoderRearRightDrive,
                m_encoderTopShooter,
                m_encoderBottomShooter);

  V_FWD = c_joyStick.GetRawAxis(1) * -1;
  V_STR = c_joyStick.GetRawAxis(0);
  V_RCW = c_joyStick.GetRawAxis(4);

  //PDP top shooter port 13
  //PDP bottom shooter port 12
  PDP_Current_UpperShooter = PDP.GetCurrent(13);
  PDP_Current_LowerShooter = PDP.GetCurrent(12);
  if(abs(PDP_Current_LowerShooter - PDP_Current_LowerShooter_last) > 2 || abs(PDP_Current_UpperShooter - PDP_Current_UpperShooter_last) > 2)
  {
    BallsShot += 1;
  }
  PDP_Current_UpperShooter_last = PDP_Current_UpperShooter;
  PDP_Current_LowerShooter_last = PDP_Current_LowerShooter;


  /* Let's place a deadband around the joystick readings */
  V_FWD = DesiredSpeed(V_FWD);
  V_STR = DesiredSpeed(V_STR);
  V_RCW = DesiredSpeed(V_RCW);


 //turning rotatemode on/off & setting desired angle
  if (c_joyStick.GetRawButton(4)) {
    rotateMode = true;
    desiredAngle = 90;
  }
  else if (c_joyStick.GetRawButton(3)) {
    rotateMode = true;
    desiredAngle = 0;
  }
  else if (c_joyStick.GetRawButton(5)) {
    rotateMode = true;
    desiredAngle = 67.5;
  }


//error calculation section
  double errorCalc = desiredAngle - gyro_yawangledegrees;


  if (rotateMode == true && fabs(errorCalc) <= 1 && rotateDeBounce <= 0.25) {
    rotateMode = true;
    rotateDeBounce += 0.01;
  }
  else if (rotateMode == true && fabs(errorCalc) <= 1 && rotateDeBounce >= 0.25) {
    rotateMode = false;
    rotateDeBounce = 0;
  }

  if (rotateMode == true)
    {
    V_RCW = Control_PID(desiredAngle,
                        gyro_yawangledegrees,
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

  L_temp = V_FWD * cos(gyro_yawanglerad) + V_STR * sin(gyro_yawanglerad);
  V_STR = -V_FWD * sin(gyro_yawanglerad) + V_STR * cos(gyro_yawanglerad);
  V_FWD = L_temp;

  //Ws1: fr, Ws2: fl, ws3: rl, ws4: rr
  L_A = V_STR - V_RCW * (C_L/C_R);
  L_B = V_STR + V_RCW * (C_L/C_R);
  L_C = V_FWD - V_RCW * (C_W/C_R);
  L_D = V_FWD + V_RCW * (C_W/C_R);

  V_WS[E_FrontRight] = pow((L_B * L_B + L_C * L_C), 0.5);
  V_WS[E_FrontLeft]  = pow((L_B * L_B + L_D * L_D), 0.5);
  V_WS[E_RearLeft]   = pow((L_A * L_A + L_D * L_D), 0.5);
  V_WS[E_RearRight]  = pow((L_A * L_A + L_C * L_C), 0.5);

  V_WA[E_FrontRight] = atan2(L_B, L_C) *180/C_PI;
  V_WA[E_FrontLeft]  = atan2(L_B, L_D) *180/C_PI;
  V_WA[E_RearLeft]   = atan2(L_A, L_D) *180/C_PI;
  V_WA[E_RearRight]  = atan2(L_A, L_C) *180/C_PI;

  L_Max = V_WS[E_FrontRight];

  if (V_WS[E_FrontLeft] > L_Max) {
    L_Max = V_WS[E_FrontLeft];
  }
 if (V_WS[E_RearLeft] > L_Max) {
    L_Max = V_WS[E_RearLeft];
  }
  if (V_WS[E_RearRight] > L_Max) {
    L_Max = V_WS[E_RearRight];
  }
  if (L_Max > 1) {
      V_WS[E_FrontRight] /= L_Max;
      V_WS[E_FrontLeft]  /= L_Max;
      V_WS[E_RearLeft]   /= L_Max;
      V_WS[E_RearRight]  /= L_Max;
  }

  L_Gain = 0.1;
  if (c_joyStick.GetRawAxis(3) > L_Gain)
    {
    L_Gain = c_joyStick.GetRawAxis(3);
    }

  V_WS[E_FrontRight] *= (K_WheelMaxSpeed * L_Gain);
  V_WS[E_FrontLeft]  *= (K_WheelMaxSpeed * L_Gain);
  V_WS[E_RearLeft]   *= (K_WheelMaxSpeed * L_Gain);
  V_WS[E_RearRight]  *= (K_WheelMaxSpeed * L_Gain);

  for (index = E_FrontLeft;
       index < E_RobotCornerSz;
       index = T_RobotCorner(int(index) + 1))
  {
    L_WA_FWD = DtrmnEncoderRelativeToCmnd(V_WA[index],
                                          V_WheelAngleFwd[index]);

    L_WA_FWD_Delta = fabs(V_WA[index] - L_WA_FWD);

    L_WA_REV = DtrmnEncoderRelativeToCmnd(V_WA[index],
                                          V_WheelAngleRev[index]);

    L_WA_REV_Delta = fabs(V_WA[index] - L_WA_REV);

    if (L_WA_FWD_Delta <= L_WA_REV_Delta)
      {
        V_WheelAngleArb[index] = L_WA_FWD;
      }
    else
      {
        V_WheelAngleArb[index] = L_WA_REV;
        V_WS[index] *= (-1); // Need to flip sign of drive wheel to acount for reverse direction
      }
  }

  //8.31 : 1
  //11.9 m/s

  // V_RobotInit = true;
  /* Ok, let's check to see if we are in init: */
    if (V_RobotInit == true)
      {
      V_RobotInit = false;
      for (index = E_FrontLeft;
           index < E_RobotCornerSz;
           index = T_RobotCorner(int(index) + 1))
        {
        V_WS[index] = 0;
        V_WA[index] = 0;
        V_WheelAngleArb[index] = V_WheelAngleFwd[index];
        if (fabs(V_WheelAngleArb[index]) > K_InitAngle)
          {
          V_RobotInit = true;
          }
        }
      }
    // V_RobotInit = true;

    for (index = E_FrontLeft;
         index < E_RobotCornerSz;
         index = T_RobotCorner(int(index) + 1))
      {
      V_WheelAngleCmnd[index] =  Control_PID( V_WA[index],
                                              V_WheelAngleArb[index],
                                             &V_WheelAngleError[index],
                                             &V_WheelAngleIntegral[index],
                                              K_WheelAnglePID_Gx[E_P_Gx],
                                              K_WheelAnglePID_Gx[E_I_Gx],
                                              K_WheelAnglePID_Gx[E_D_Gx],
                                              K_WheelAnglePID_Gx[E_P_Ul],
                                              K_WheelAnglePID_Gx[E_P_Ll],
                                              K_WheelAnglePID_Gx[E_I_Ul],
                                              K_WheelAnglePID_Gx[E_I_Ll],
                                              K_WheelAnglePID_Gx[E_D_Ul],
                                              K_WheelAnglePID_Gx[E_D_Ll],
                                              K_WheelAnglePID_Gx[E_Max_Ul],
                                              K_WheelAnglePID_Gx[E_Max_Ll]);

      V_WheelSpeedCmnd[index] = Control_PID( V_WS[index],
                                             V_WheelVelocity[index],
                                            &V_WheelSpeedError[index],
                                            &V_WheelSpeedIntergral[index],
                                             K_WheelSpeedPID_Gx[E_P_Gx],
                                             K_WheelSpeedPID_Gx[E_I_Gx],
                                             K_WheelSpeedPID_Gx[E_D_Gx],
                                             K_WheelSpeedPID_Gx[E_P_Ul],
                                             K_WheelSpeedPID_Gx[E_P_Ll],
                                             K_WheelSpeedPID_Gx[E_I_Ul],
                                             K_WheelSpeedPID_Gx[E_I_Ll],
                                             K_WheelSpeedPID_Gx[E_D_Ul],
                                             K_WheelSpeedPID_Gx[E_D_Ll],
                                             K_WheelSpeedPID_Gx[E_Max_Ul],
                                             K_WheelSpeedPID_Gx[E_Max_Ll]);
      }
    //Ws1: fr, Ws2: fl, ws3: rl, ws4: rr

    frc::SmartDashboard::PutBoolean("rotateMode",rotateMode);
    frc::SmartDashboard::PutNumber("errorCalc",errorCalc);
    frc::SmartDashboard::PutNumber("desiredAngle",desiredAngle);
    frc::SmartDashboard::PutNumber("Gyro Angle Deg", gyro_yawangledegrees);
    frc::SmartDashboard::PutNumber("Gyro Angle Rad", gyro_yawanglerad);
    frc::SmartDashboard::PutNumber("ROLL OVER RAD", gyro_rolloverrad);

    frc::SmartDashboard::PutNumber("Front Left Wheel Velocity",(V_WheelVelocity[E_FrontLeft]));
    frc::SmartDashboard::PutNumber("Front Right Wheel Velocity",(V_WheelVelocity[E_FrontRight]));
    frc::SmartDashboard::PutNumber("Rear Left Wheel Velocity",(V_WheelVelocity[E_RearLeft]));
    frc::SmartDashboard::PutNumber("Rear Right Wheel Velocity",(V_WheelVelocity[E_RearRight]));

    frc::SmartDashboard::PutNumber("STR", V_STR);
    frc::SmartDashboard::PutNumber("FWD", V_FWD);
    frc::SmartDashboard::PutNumber("RCW", V_RCW);

    frc::SmartDashboard::PutNumber("Wheel angle FR", V_WheelAngleArb[E_FrontRight]);
    frc::SmartDashboard::PutNumber("Wheel angle FL", V_WheelAngleArb[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("Wheel angle RR", V_WheelAngleArb[E_RearRight]);
    frc::SmartDashboard::PutNumber("Wheel angle RL", V_WheelAngleArb[E_RearLeft]);

    frc::SmartDashboard::PutNumber("WS_FR", V_WS[E_FrontRight]);
    frc::SmartDashboard::PutNumber("WS_FL", V_WS[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("WS_RL", V_WS[E_RearLeft]);
    frc::SmartDashboard::PutNumber("WS_RR", V_WS[E_RearRight]);

    frc::SmartDashboard::PutNumber("WA_FR", V_WA[E_FrontRight]);
    frc::SmartDashboard::PutNumber("WA_FL", V_WA[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("WA_RL", V_WA[E_RearLeft]);
    frc::SmartDashboard::PutNumber("WA_RR", V_WA[E_RearRight]);

    frc::SmartDashboard::PutBoolean("RobotInit",  V_RobotInit);

    //Shooter mech
    frc::SmartDashboard::PutNumber("Top Speed", V_ShooterSpeedCurr[E_TopShooter]);
    frc::SmartDashboard::PutNumber("Bottom Speed", V_ShooterSpeedCurr[E_BottomShooter]);

    SpeedRecommend = (distanceTarget * sqrt(-9.807 / (2 * cos(35 * C_Deg2Rad) * cos(35 * C_Deg2Rad) * (1.56845 - (distanceTarget * tan(35 * C_Deg2Rad))))));
    frc::SmartDashboard::PutNumber("Recommended Speed", SpeedRecommend);

    double L_RequestedSpeed;

    //Shooter mech controls
    // V_ShooterSpeedDesired[E_TopShooter]    = c_joyStick2.GetRawAxis(1) * 10000;
    // V_ShooterSpeedDesired[E_BottomShooter] = c_joyStick2.GetRawAxis(5) * 10000;

//    V_ShooterSpeedDesired[E_TopShooter]    = frc::SmartDashboard::GetNumber("Speed Desired Top", 0);
//    V_ShooterSpeedDesired[E_BottomShooter] = frc::SmartDashboard::GetNumber("Speed Desired Bottom", 0);

    L_RequestedSpeed = frc::SmartDashboard::GetNumber("Speed Desired Top", 0);

    V_ShooterSpeedDesired[E_TopShooter] =RampTo(L_RequestedSpeed,
                                                V_ShooterSpeedDesired[E_TopShooter],
                                                30);

    L_RequestedSpeed = frc::SmartDashboard::GetNumber("Speed Desired Bottom", 0);

    V_ShooterSpeedDesired[E_BottomShooter] =RampTo(L_RequestedSpeed,
                                                   V_ShooterSpeedDesired[E_BottomShooter],
                                                   30);

    double upper_P_Gx = frc::SmartDashboard::GetNumber("Upper_P_Gx", 0);
    double upper_I_Gx = frc::SmartDashboard::GetNumber("Upper_I_Gx", 0);
    double upper_D_Gx = frc::SmartDashboard::GetNumber("Upper_D_Gx", 0);
    double upper_I_Zone = frc::SmartDashboard::GetNumber("Upper_I_Zone", 0);
    double upper_FF = frc::SmartDashboard::GetNumber("Upper_FF", 0);
    double upper_Max = frc::SmartDashboard::GetNumber("Upper_Max_Limit", 0);
    double upper_Min = frc::SmartDashboard::GetNumber("Upper_Min_Limit", 0);

    double lower_P_Gx = frc::SmartDashboard::GetNumber("Lower_P_Gx", 0);
    double lower_I_Gx = frc::SmartDashboard::GetNumber("Lower_I_Gx", 0);
    double lower_D_Gx = frc::SmartDashboard::GetNumber("Lower_D_Gx", 0);
    double lower_I_Zone =frc::SmartDashboard::GetNumber("Lower_I_Zone", 0);
    double lower_FF = frc::SmartDashboard::GetNumber("Lower_FF", 0);
    double lower_Max = frc::SmartDashboard::GetNumber("Lower_Max_Limit", 0);
    double lower_Min = frc::SmartDashboard::GetNumber("Lower_Min_Limit", 0);

    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);

    if(upper_P_Gx != Upper_P_Gx) { m_topShooterpid.SetP(upper_P_Gx); Upper_P_Gx = upper_P_Gx; }
    if(upper_I_Gx != Upper_I_Gx) { m_topShooterpid.SetI(upper_I_Gx); Upper_I_Gx = upper_I_Gx; }
    if(upper_D_Gx != Upper_D_Gx) { m_topShooterpid.SetD(upper_D_Gx); Upper_D_Gx = upper_D_Gx; }
    if(upper_I_Zone != Upper_I_Zone) { m_topShooterpid.SetIZone(upper_I_Zone); Upper_I_Zone = upper_I_Zone; }
    if(upper_FF != Upper_FF) { m_topShooterpid.SetFF(upper_FF); Upper_FF = upper_FF; }
    if ((upper_Max != Upper_Max) || (upper_Min != Upper_Min))
    {
      m_topShooterpid.SetOutputRange(upper_Min, upper_Max);
      Upper_Min = upper_Min; Upper_Max = upper_Max;
    }

    if(lower_P_Gx != Lower_P_Gx) { m_bottomShooterpid.SetP(lower_P_Gx); Lower_P_Gx = lower_P_Gx; }
    if(lower_I_Gx != Lower_I_Gx) { m_bottomShooterpid.SetI(lower_I_Gx); Lower_I_Gx = lower_I_Gx; }
    if(lower_D_Gx != Lower_D_Gx) { m_bottomShooterpid.SetD(lower_D_Gx); Lower_D_Gx = lower_D_Gx; }
    if(lower_I_Zone != Lower_I_Zone) { m_bottomShooterpid.SetIZone(lower_I_Zone); Lower_I_Zone = lower_I_Zone; }
    if(lower_FF != Lower_FF) { m_bottomShooterpid.SetFF(lower_FF); Lower_FF = lower_FF; }
    if ((lower_Max != Lower_Max) || (lower_Min != Lower_Min))
    {
      m_bottomShooterpid.SetOutputRange(lower_Min, lower_Max);
      Lower_Min = lower_Min; Lower_Max = lower_Max;
    }

    if((p != kP)) { m_liftpid.SetP(p); kP = p; }
    if((i != kI)) { m_liftpid.SetI(i); kI = i; }
    if((d != kD)) { m_liftpid.SetD(d); kD = d; }
    if((iz != kIz)) { m_liftpid.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_liftpid.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput))
    {
      m_liftpid.SetOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max;
    }
    m_liftpid.SetIMaxAccum(0.1);
    double desiredlevel = frc::SmartDashboard::GetNumber("Desired Level", 0);

    frc::SmartDashboard::PutNumber("Postion", m_encoderLift.GetPosition());

    //m_liftpid.SetReference(desiredlevel , rev::ControlType::kPosition);
    //m_liftMotor.Set(c_joyStick2.GetRawAxis(1) * 0.25);
    frc::SmartDashboard::PutNumber("Upper Velocity", m_encoderTopShooter.GetVelocity());
    frc::SmartDashboard::PutNumber("Lower Velocity", m_encoderBottomShooter.GetVelocity());

    m_topShooterpid.SetReference(V_ShooterSpeedDesired[E_TopShooter], rev::ControlType::kVelocity);
    m_bottomShooterpid.SetReference(V_ShooterSpeedDesired[E_BottomShooter], rev::ControlType::kVelocity);

    m_fortuneWheel.Set(ControlMode::PercentOutput, L_FortuneMotor);

    m_frontLeftDriveMotor.Set(V_WheelSpeedCmnd[E_FrontLeft]);
    m_frontRightDriveMotor.Set(V_WheelSpeedCmnd[E_FrontRight]);
    m_rearLeftDriveMotor.Set(V_WheelSpeedCmnd[E_RearLeft]);
    m_rearRightDriveMotor.Set(V_WheelSpeedCmnd[E_RearRight]);

    // m_frontLeftDriveMotor.Set(0);
    // m_frontRightDriveMotor.Set(0);
    // m_rearLeftDriveMotor.Set(0);
    // m_rearRightDriveMotor.Set(0);

    m_frontLeftSteerMotor.Set(V_WheelAngleCmnd[E_FrontLeft] * (-1));
    m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight] * (-1));
    m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft] * (-1));
    m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight] * (-1));

    // m_frontLeftSteerMotor.Set(0);
    // m_frontRightSteerMotor.Set(0);
    // m_rearLeftSteerMotor.Set(0);
    // m_rearRightSteerMotor.Set(0);

    //Pneumatic Lift
    if(c_joyStick2.GetRawButton(5))
    {
      lift.Set(frc::DoubleSolenoid::Value::kReverse);
      //go up
    }
    else if(c_joyStick2.GetRawButton(6))
    {
      lift.Set(frc::DoubleSolenoid::Value::kForward);
      //go down
    }
    else
    {
      lift.Set(frc::DoubleSolenoid::Value::kOff);
    }

    //Intake Control
    if(c_joyStick2.GetRawButton(2))
    {
      intake.Set(frc::DoubleSolenoid::Value::kForward);
      m_intake.Set(ControlMode::PercentOutput, 0.5);
    }
    else
    {
      intake.Set(frc::DoubleSolenoid::Value::kOff);
      m_intake.Set(ControlMode::PercentOutput, 0);
    }

    if(c_joyStick2.GetRawButton(1))
    {
      m_belt.Set(ControlMode::PercentOutput, 1);
    }
    else
    {
      m_belt.Set(ControlMode::PercentOutput, 0);
    }
    /*
      Finds distance from robot to specified target.
      Numerator depends upon camera height relative to target for target distance,
      and camera height relative to ground for ball distance.
      Make sure it's in meters.
    */
    distanceTarget     = 157.8 / tan((targetPitch0.GetDouble(0)) * (C_Deg2Rad));
    distanceBall       = 47  / tan((targetPitch1.GetDouble(0)) * (-C_Deg2Rad));

    //Finds robot's distance from target's center view.
    distanceFromTargetCenter = distanceTarget * sin((90 - targetYaw0.GetDouble(0)) * C_Deg2Rad);
    distanceFromBallCenter   = distanceBall   * sin((90 - targetYaw1.GetDouble(0)) * C_Deg2Rad);

    frc::SmartDashboard::PutNumber("distanceTarget", distanceTarget);

    // Toggle for target adjust
    if(c_joyStick.GetRawButton(1) == true)
    {
        visionInit(vision0, ledLight, inst);
        visionStart1 = true;
    }
    if(c_joyStick.GetRawButton(1) == false && visionStart1 == true)
    {
        visionOff(vision0, ledLight, inst, visionStart1, visionStart2, activeVisionAngle0, activeVisionDistance0);
        V_RCW = 0;
        visionStart1 = false;
    }
    if(visionStart1 == true)
    {
        V_RCW = AutoTarget(targetYaw0, distanceTarget, 0, activeVisionAngle0, activeVisionDistance0, desiredVisionAngle0, desiredVisionDistance0);
    }

    //Toggle for autoshoot



    //  if(c_joyStick.GetRawButton(2) == true)
    //  {
    //      visionInit(vision0, ledLight);
    //      visionStart[2] = true;
    //  }

    //  if(c_joyStick.GetRawButton(3) == true)
    //  {
    //     visionRequest = true;
    //  }

    //  if(c_joyStick.GetRawButton(4) == true)
    //  {
    //     visionStart[1] = false; visionStart[2] = false;
    //  }

    //  if(visionStart[1] == true)
    //  {
    //     visionRun(targetYaw0, distanceFromTargetCenter, high, vision0, ledLight);
    //  }

    // if(visionStart[2] == true)
    // {
    //     visionRun(targetYaw1, -distanceFromBallCenter, ball, vision1, ledLight);
    // }

    frc::Wait(C_ExeTime);
}


/******************************************************************************
 * Function:     TestPeriodic
 *
 * Description:  Called during the test phase initiated on the driver station.
 ******************************************************************************/
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
