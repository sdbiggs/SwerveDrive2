/*
 * Team 5561 2020 Code
 *
 * This code runs the 2020 robot which is capable of the following:
 * - Swerve Drive
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
#include "SwerveDrive.hpp"

double V_FWD;
double V_STR;
double V_RCW;
double V_WS[E_RobotCornerSz];
double V_WA[E_RobotCornerSz];
double V_WA_Prev[E_RobotCornerSz];

double V_WheelAngleCmnd[E_RobotCornerSz];
double V_WheelAngleError[E_RobotCornerSz];
double V_WheelAngleIntegral[E_RobotCornerSz];
double V_WheelSpeedCmnd[E_RobotCornerSz];
double V_WheelSpeedError[E_RobotCornerSz];
double V_WheelSpeedIntergral[E_RobotCornerSz];

bool   V_RobotInit;

double V_WheelAngleCase[E_RobotCornerSz];  // For trouble shooting where the angle is coming from

bool rotatemode;

double V_ShooterSpeedCurr[E_RoboShooter];
double V_ShooterSpeedCmnd[E_RoboShooter];
double V_ShooterSpeedDesired[E_RoboShooter];
double V_ShooterSpeedIntegral[E_RoboShooter];
double V_ShooterSpeedError[E_RoboShooter];
bool   V_ShooterRequest[2] {false, false};

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

const double deg2rad = 0.017453292519943295;
const double shooterWheelRotation = (2.5555555555555555555555555555555555555555555555 * (2 * C_PI));
double       distanceTarget;
double       distanceBall;
double       distanceFromTargetCenter;
double       distanceFromBallCenter;
double       desiredVisionAngle0;
double       desiredVisionDistance0;
bool         activeVisionAngle0;
bool         activeVisionDistance0;
bool         visionRequest;
bool         visionStart[2] {false, false};
enum         VisionAuton {strafe, rotate, complete};
enum         VisionTarget{high, ball};
char         autonChoose;
frc::LiveWindow *lw = frc::LiveWindow::GetInstance();
std::shared_ptr<NetworkTable> vision;
// nt::NetworkTableInstance inst;
nt::NetworkTableEntry driverMode;

/******************************************************************************
 * Function:     Control_PID
 *
 * Description:  This function provides PID control.  This will also limit the
 *               the three controllers (P I D) by the calibrated thresholds.
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

  return (L_CriteriaMet);
  }

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
  double L_FWD;
  double L_STR;
  double L_RCW;

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

  /* Let's place a deadband around the joystick readings */
  L_FWD = DesiredSpeed(c_joyStick.GetRawAxis(1) * -1);
  L_STR = DesiredSpeed(c_joyStick.GetRawAxis(0));
  L_RCW = DesiredSpeed(c_joyStick.GetRawAxis(4));
  L_Gain = c_joyStick.GetRawAxis(3);

  SwerveDriveWheelOutput(L_FWD,
                        L_STR,
                        L_RCW,
                            L_Gain,
                            &V_WheelAngleArb[0],
                            &V_WA[0],
                            &V_WS[0]);
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

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
