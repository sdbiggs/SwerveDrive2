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

double V_FWD;
double V_STR;
double V_RCW;
double V_WS[E_RobotCornerSz];
double V_WA[E_RobotCornerSz];
double V_WA_Prev[E_RobotCornerSz];

double V_ShooterSpeedCurr[E_RoboShooter];
double V_ShooterSpeedCmnd[E_RoboShooter];
double V_ShooterSpeedDesired[E_RoboShooter];
double V_ShooterSpeedIntegral[E_RoboShooter];
double V_ShooterSpeedError[E_RoboShooter];
bool   V_ShooterRequest[2] {false, false};
const double shooterWheelRotation = (2.5555555555555555555555555555555555555555555555 * (2 * C_PI));

double V_WheelAngleCmnd[E_RobotCornerSz];
double V_WheelAngleError[E_RobotCornerSz];
double V_WheelAngleIntegral[E_RobotCornerSz];
double V_WheelSpeedCmnd[E_RobotCornerSz];
double V_WheelSpeedError[E_RobotCornerSz];
double V_WheelSpeedIntergral[E_RobotCornerSz];

bool   V_RobotInit;


double V_WheelSpeedError[E_RobotCornerSz];
double V_WheelAngleCase[E_RobotCornerSz];
double V_FWD;
double V_STR;
double V_RCW;
double V_GAIN;


frc::LiveWindow *lw = frc::LiveWindow::GetInstance();
std::shared_ptr<NetworkTable> vision;
nt::NetworkTableInstance inst;
nt::NetworkTableEntry driverMode;


frc::LiveWindow *lw = frc::LiveWindow::GetInstance();
std::shared_ptr<NetworkTable> vision;
nt::NetworkTableInstance inst;
nt::NetworkTableEntry driverMode;

/******************************************************************************
 * Function:     CriteriaMet
 *
 * Description:  This function checks to see if certain criteria is met.
 ******************************************************************************/
bool CriteriaMet(double  L_Desired,
                 double  L_Current,
                 double  L_AllowedError)
  {
  bool L_CriteriaMet = true;

  if (fabs(L_Current - L_Desired) <= L_AllowedError)
    {
    L_CriteriaMet = false;
    }

  GyroRobotInit();    
  inst = nt::NetworkTableInstance::Create();
  inst.StartClient("10.55.61.24");
  inst.StartDSClient();

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

    V_RobotInit = false;

    GyroRobotInit();
    inst = nt::NetworkTableInstance::Create();
    inst.StartClient("10.55.61.24");
    inst.StartDSClient();

    vision = inst.GetTable("chameleon-vision/Scotty");

    driverMode = vision->GetEntry("driver_mode");
}

void Robot::RobotPeriodic() {}


/******************************************************************************
 * Function:     AutonomousInit
 *
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

  GyroTeleInit();
}

  SmartDashboard::PutNumber("Front Left Wheel Velocity",(V_WheelVelocity[E_FrontLeft]));
  SmartDashboard::PutNumber("Front Right Wheel Velocity",(V_WheelVelocity[E_FrontRight]));
  SmartDashboard::PutNumber("Rear Left Wheel Velocity",(V_WheelVelocity[E_RearLeft]));
  SmartDashboard::PutNumber("Rear Right Wheel Velocity",(V_WheelVelocity[E_RearRight]));

  SmartDashboard::PutNumber("RPM Desire Front Left", V_WheelRpmCmnd[E_FrontLeft]);
  SmartDashboard::PutNumber("RPM Desire Front Right", V_WheelRpmCmnd[E_FrontRight]);
  SmartDashboard::PutNumber("RPM Desire Rear Left", V_WheelRpmCmnd[E_RearLeft]);
  SmartDashboard::PutNumber("RPM Desire Rear Right", V_WheelRpmCmnd[E_RearRight]);

  SmartDashboard::PutNumber("Angle Desire Front Left", V_DesiredWheelAngle[E_FrontLeft]);
  SmartDashboard::PutNumber("Angle Desire Front Right", V_DesiredWheelAngle[E_FrontRight]);
  SmartDashboard::PutNumber("Angle Desire Rear Left", V_DesiredWheelAngle[E_RearLeft]);
  SmartDashboard::PutNumber("Angle Desire Rear Right", V_DesiredWheelAngle[E_RearRight]);

  //8.31 : 1 
  //11.9 m/s

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

  ColorSensor(false);
  Gyro();

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
                m_encoderRearRightDrive);

  V_FWD = c_joyStick.GetRawAxis(1) * -1;
  V_STR = c_joyStick.GetRawAxis(0);
  V_RCW = c_joyStick.GetRawAxis(4);
  V_GAIN = c_joyStick.GetRawAxis(3);

  /* Let's place a deadband around the joystick readings */
  V_FWD = DesiredSpeed(V_FWD);
  V_STR = DesiredSpeed(V_STR);
  V_RCW = DesiredSpeed(V_RCW);

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
      V_WS[E_FrontLeft] /= L_Max;
      V_WS[E_RearLeft] /= L_Max;
      V_WS[E_RearRight] /= L_Max;
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
      V_WheelAngleCmnd[index] =  Control_PID(V_WA[index],
                                             V_WheelAngleArb[index],
                                             &V_WheelAngleError[index],
                                             &V_WheelAngleIntegral[index],
                                              0.007, // P Gx 0.0045
                                              0.0005, // I Gx 0001
                                              0.0000005, // D Gx
                                              0.4, // P UL
                                             -0.4, // P LL
                                              0.12, // I UL
                                             -0.12, // I LL
                                              0.5, // D UL
                                             -0.5, // D LL
                                              0.9, // Max upper
                                             -0.9); // Max lower

      V_WheelSpeedCmnd[index] = Control_PID(V_WS[index],
                                            V_WheelVelocity[index],
                                            &V_WheelSpeedError[index],
                                            &V_WheelSpeedIntergral[index],
                                            0.0055,   // P Gx 0.0038 0.005
                                            0.0009, // I Gx 0.0029  0.001
                                            0.00000005, // D Gx
                                            0.9, // P UL
                                            -0.9, // P LL
                                            0.5, // I UL
                                            -0.5, // I LL
                                            0.2, // D UL
                                            -0.2, // D LL
                                            1.0, // Max upper
                                            -1.0);
      }
    //Ws1: fr, Ws2: fl, ws3: rl, ws4: rr

    frc::SmartDashboard::PutNumber("FL Case",(V_WheelAngleCase[E_FrontLeft]));
    frc::SmartDashboard::PutNumber("FR Case",(V_WheelAngleCase[E_FrontRight]));
    frc::SmartDashboard::PutNumber("RL Case",(V_WheelAngleCase[E_RearLeft]));
    frc::SmartDashboard::PutNumber("RR Case",(V_WheelAngleCase[E_RearRight]));

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

    frc::SmartDashboard::PutNumber("Drive Intergral Front Left", V_WheelSpeedIntergral[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("Drive Intergral Front Right", V_WheelSpeedIntergral[E_FrontRight]);
    frc::SmartDashboard::PutNumber("Drive Intergral Rear Left", V_WheelSpeedIntergral[E_RearLeft]);
    frc::SmartDashboard::PutNumber("Drive Intergral Rear Right", V_WheelSpeedIntergral[E_RearRight]);

    frc::SmartDashboard::PutNumber("Wheel angle integral FR", V_WheelAngleIntegral[E_FrontRight]);
    frc::SmartDashboard::PutNumber("Wheel angle integral FL", V_WheelAngleIntegral[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("Wheel angle integral RR", V_WheelAngleIntegral[E_RearRight]);
    frc::SmartDashboard::PutNumber("Wheel angle integral RL", V_WheelAngleIntegral[E_RearLeft]);

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

    frc::SmartDashboard::PutNumber("WA Loop Count FL", V_WA_Loopcount[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("Gyro Angle Deg", gyro_yawangledegrees);
    frc::SmartDashboard::PutNumber("Gyro Angle Rad", gyro_yawanglerad);
    frc::SmartDashboard::PutNumber("ROLL OVER RAD", gyro_rolloverrad);

    frc::SmartDashboard::PutNumber("Front Left Wheel Velocity",(V_WheelVelocity[E_FrontLeft]));
    frc::SmartDashboard::PutNumber("Front Right Wheel Velocity",(V_WheelVelocity[E_FrontRight]));
    frc::SmartDashboard::PutNumber("Rear Left Wheel Velocity",(V_WheelVelocity[E_RearLeft]));
    frc::SmartDashboard::PutNumber("Rear Right Wheel Velocity",(V_WheelVelocity[E_RearRight]));

    frc::SmartDashboard::PutNumber("RPM Desire Front Left", V_WheelRpmCmnd[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("RPM Desire Front Right", V_WheelRpmCmnd[E_FrontRight]);
    frc::SmartDashboard::PutNumber("RPM Desire Rear Left", V_WheelRpmCmnd[E_RearLeft]);
    frc::SmartDashboard::PutNumber("RPM Desire Rear Right", V_WheelRpmCmnd[E_RearRight]);

    frc::SmartDashboard::PutNumber("Angle Desire Front Left", V_DesiredWheelAngle[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("Angle Desire Front Right", V_DesiredWheelAngle[E_FrontRight]);
    frc::SmartDashboard::PutNumber("Angle Desire Rear Left", V_DesiredWheelAngle[E_RearLeft]);
    frc::SmartDashboard::PutNumber("Angle Desire Rear Right", V_DesiredWheelAngle[E_RearRight]);

    frc::SmartDashboard::PutNumber("STR", V_JoystickAxisStrafe);
    frc::SmartDashboard::PutNumber("FWD", V_JoystickAxisForward);
    frc::SmartDashboard::PutNumber("RCW", V_JoystickAxisRotate);

    frc::SmartDashboard::PutBoolean("Wheel Delay",  V_WheelSpeedDelay);
    frc::SmartDashboard::PutBoolean("RobotInit",  V_RobotInit);
    frc::SmartDashboard::PutNumber("Desired Wheel Angle",  V_DesiredWheelAngle[E_FrontLeft]);

    frc::SmartDashboard::PutNumber("Angle Desire Front Left", V_DesiredWheelAngle[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("Angle Desire Front Right", V_DesiredWheelAngle[E_FrontRight]);
    frc::SmartDashboard::PutNumber("Angle Desire Rear Left", V_DesiredWheelAngle[E_RearLeft]);
    frc::SmartDashboard::PutNumber("Angle Desire Rear Right", V_DesiredWheelAngle[E_RearRight]);

    frc::SmartDashboard::PutBoolean("RobotInit",  V_RobotInit);

    if(c_joyStick2.GetRawButton(4))
    {
      m_intake.Set(ControlMode::PercentOutput, 0.5);
    } else {
      m_intake.Set(ControlMode::PercentOutput, 0);
    }
    
        //Shooter mech
    T_RoboShooter dex;
    V_ShooterSpeedCurr[E_TopShooter]    = (m_encoderTopShooter.GetVelocity()    * shooterWheelRotation) * 0.3191858136047229930278045677412;
    V_ShooterSpeedCurr[E_BottomShooter] = (m_encoderBottomShooter.GetVelocity() * shooterWheelRotation) * 0.2393893602035422447708534258059;

    frc::SmartDashboard::PutNumber("Top Speed", V_ShooterSpeedCurr[E_TopShooter]);
    frc::SmartDashboard::PutNumber("Bottom Speed", V_ShooterSpeedCurr[E_BottomShooter]);

    
    //Shooter mech controls
    V_ShooterSpeedCmnd[E_TopShooter]    = c_joyStick2.GetRawAxis(1);
    V_ShooterSpeedCmnd[E_BottomShooter] = c_joyStick2.GetRawAxis(5);


    
    if (c_joyStick2.GetRawButton(1) == true)
    {
        V_ShooterSpeedCmnd[E_TopShooter] = -.75;
        V_ShooterSpeedCmnd[E_BottomShooter] = -1.0;
    }

    // if (c_joyStick2.GetRawButton(2) == true)
    // {
    //     V_ShooterRequest[1] = true;
    // }

    // if (c_joyStick2.GetRawButton(3) == true)
    // {
    //     V_ShooterRequest[1] = false; V_ShooterRequest[2] = false;
    // }


    //Shooter mech logic
    // for (dex = E_TopShooter;
    //      dex < E_RoboShooter;
    //      dex = T_RoboShooter(int(dex) + 1))
    // {
    //     V_ShooterSpeedCmnd[dex] = Control_PID(V_ShooterSpeedDesired[dex],
    //                                           V_ShooterSpeedCurr[dex],
    //                                           &V_ShooterSpeedError[dex],
    //                                           &V_ShooterSpeedIntegral[dex],
    //                                           0.0069, // P Gx
    //                                           0.000069, // I Gx
    //                                           0.0, // D Gx
    //                                           0.069, // P UL
    //                                          -0.069, // P LL
    //                                           0.0069, // I UL
    //                                          -0.0069, // I LL
    //                                           0.0, // D UL
    //                                          -0.0, // D LL
    //                                           0.69, // Max upper
    //                                          -0.69); // Max lower
    // }


    //Shooter mech w/ vision assist
    // if (V_ShooterRequest[1] == true)
    // {
    //     for (dex = E_TopShooter;
    //          dex < E_RoboShooter;
    //          dex = T_RoboShooter(int(dex) + 1))
    //     {
    //          V_ShooterSpeedDesired[dex] = (distanceTarget * sqrt(-9.807 / (2 * cos(C_PI / 4) * cos(C_PI / 4) * (1.56845 - (distanceTarget * tan(C_PI / 4))))));
    //          V_ShooterRequest[2] = true;
    //     }
    // }

    // if (V_ShooterRequest[2] == true)
    // {
    //     //starrt loading balls
    // }



    m_topShooterMotor.Set(V_ShooterSpeedCmnd[E_TopShooter]);
    m_bottomShooterMotor.Set(V_ShooterSpeedCmnd[E_BottomShooter]);  

    m_frontLeftDriveMotor.Set(V_WheelSpeedCmnd[E_FrontLeft]);
    m_frontRightDriveMotor.Set(V_WheelSpeedCmnd[E_FrontRight]);
    m_rearLeftDriveMotor.Set(V_WheelSpeedCmnd[E_RearLeft]);
    m_rearRightDriveMotor.Set(V_WheelSpeedCmnd[E_RearRight]);

    // m_frontLeftDriveMotor.Set(0);
    // m_frontRightDriveMotor.Set(0);
    // m_rearLeftDriveMotor.Set(0);
    // m_rearRightDriveMotor.Set(0);

    m_frontLeftSteerMotor.Set (V_WheelAngleCmnd[E_FrontLeft] * (-1));
    m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight] * (-1));
    m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft] * (-1));
    m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight] * (-1));

    // m_frontLeftSteerMotor.Set(0);
    // m_frontRightSteerMotor.Set(0);
    // m_rearLeftSteerMotor.Set(0);
    // m_rearRightSteerMotor.Set(0);

    frc::Wait(0.01);
}


/******************************************************************************
 * Function:     TestPeriodic
 *
 * Description:  Called druing the test phase initiated on the driver station.
 ******************************************************************************/
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
