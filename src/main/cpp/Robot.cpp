/*
 * Team 5561 2020 Code
 *
 * This code runs the 2020 robot which is capable of the following:
 * - Swerve Drive (beta 02/10/2020)
 * - Shooting balls (beta 02/15/2020)
 *
 * */

//NOTE: Set this to TEST for testing of speeds and PID gains.  Set to COMP for competion
#define TEST
//NOTE: Set this to allow Shuffleboard configuration of PIDConfig objects (Will override defaults)
#define PID_DEBUG

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/DigitalInput.h>

#include "Encoders.hpp"
#include "Enums.hpp"
#include "control_pid.hpp"
//#include "ColorSensor.hpp"
#include "Gyro.hpp"
#include "Lookup.hpp"
#include "WheelOfFortune.hpp"
#include "vision.hpp"
#include "DriveControl.hpp"
#include "AutoTarget.hpp"

#include "Utils/PIDConfig.hpp"

// T_WheelOfFortuneColor V_ColorWheelColor;

// double desiredAngle;
// double rotateDeBounce;
// double rotateErrorCalc;
// double rotateErrorIntegral;
// bool   rotateMode;
// double V_FWD;
// double V_STR;
// double V_RCW;
// double V_WS[E_RobotCornerSz];
// double V_WA[E_RobotCornerSz];

double V_WheelAngleCmnd[E_RobotCornerSz];
double V_WheelAngleError[E_RobotCornerSz];
double V_WheelAngleIntegral[E_RobotCornerSz];
double V_WheelSpeedCmnd[E_RobotCornerSz];
double V_WheelSpeedError[E_RobotCornerSz];
double V_WheelSpeedIntergral[E_RobotCornerSz];

double V_ShooterSpeedDesired[E_RoboShooter];

double V_AutoTargetAngle;
double V_AutoTargetUpperRollerSpd;
double V_AutoTargetLowerRollerSpd;
double V_AutoTargetBeltPower;

bool   V_RobotInit;

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
double       originalPosition;
bool         activeVisionAngle0;
bool         activeVisionDistance0;
bool         visionRequest;
bool         visionStart1;
bool         visionStart2;

bool         autonComplete[4];
int          beamCount;
bool         beamFullyCharged;

bool         V_AutoShootEnable;
double       V_ShooterSpeedDesiredFinalUpper;
double       V_ShooterSpeedDesiredFinalLower;

double SpeedRecommend;

double PDP_Current_UpperShooter = 0;
double PDP_Current_LowerShooter = 0;
double PDP_Current_UpperShooter_last = 0;
double PDP_Current_LowerShooter_last = 0;

double BallsShot = 0;

PIDConfig UpperShooterPIDConfig {0.0008, 0.000001, 0.0006};

frc::DigitalInput ir_sensor{1};

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
    m_liftMotor.RestoreFactoryDefaults();

    m_liftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    V_RobotInit = true;

    GyroRobotInit();

    compressor.SetClosedLoopControl(true);
    inst = nt::NetworkTableInstance::Create();
    inst.StartClient("10.55.61.24");
    inst.StartDSClient();

    vision0  = inst.GetTable("chameleon-vision/goal");
    vision1  = inst.GetTable("chameleon-vision/ColorWheel");
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

    double upper_P_Gx = .0008;
    double upper_I_Gx = .000001;
    double upper_D_Gx = .0006;
    double upper_I_Zone = 0;
    double upper_FF = 0;
    double upper_Max = 1;
    double upper_Min = -1;

    double lower_P_Gx = .0008;
    double lower_I_Gx = .000001;
    double lower_D_Gx = .0006;
    double lower_I_Zone = 0;
    double lower_FF = 0;
    double lower_Max = 1;
    double lower_Min = -1;

    m_topShooterpid.SetP(upper_P_Gx);
    m_topShooterpid.SetI(upper_I_Gx);
    m_topShooterpid.SetD(upper_D_Gx);
    m_topShooterpid.SetIZone(upper_I_Zone);
    m_topShooterpid.SetFF(upper_FF);
    m_topShooterpid.SetOutputRange(upper_Min, upper_Max);

    m_bottomShooterpid.SetP(lower_P_Gx);
    m_bottomShooterpid.SetI(lower_I_Gx);
    m_bottomShooterpid.SetD(lower_D_Gx);
    m_bottomShooterpid.SetIZone(lower_I_Zone);
    m_bottomShooterpid.SetFF(lower_FF);
    m_bottomShooterpid.SetOutputRange(lower_Min, lower_Max);

    // m_liftpid.SetP(kP);
    // m_liftpid.SetI(kI);
    // m_liftpid.SetD(kD);
    // m_liftpid.SetIZone(kIz);
    // m_liftpid.SetFF(kFF);
    // m_liftpid.SetOutputRange(kMinOutput, kMaxOutput);

    // frc::SmartDashboard::PutNumber("P Gain", kP);
    // frc::SmartDashboard::PutNumber("I Gain", kI);
    // frc::SmartDashboard::PutNumber("D Gain", kD);
    // frc::SmartDashboard::PutNumber("I Zone", kIz);
    // frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    // frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    // frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    // frc::SmartDashboard::PutNumber("Desired Level", 0);
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

    /*
      Finds distance from robot to specified target.
      Numerator depends upon camera height relative to target for target distance,
      and camera height relative to ground for ball distance.
      Make sure it's in meters.
    */
     distanceTarget     = 157.8 / tan((targetPitch0.GetDouble(0) + 15) * (C_Deg2Rad));
    //  distanceBall       = 47  / tan((targetPitch1.GetDouble(0)) * (-deg2rad));

    //Finds robot's distance from target's center view.
     distanceFromTargetCenter = (distanceTarget * sin((90 - targetYaw0.GetDouble(0)) * C_Deg2Rad) - 28.17812754);
    //  distanceFromBallCenter   = distanceBall   * sin((90 - targetYaw1.GetDouble(0)) * deg2rad);

    frc::SmartDashboard::PutNumber("distanceTarget", distanceTarget);
    frc::SmartDashboard::PutNumber("distanceFromTargetCenter", distanceFromTargetCenter);
    frc::SmartDashboard::PutNumber("targetYaw", targetYaw0.GetDouble(0));
    frc::SmartDashboard::PutNumber("targetPitch", targetPitch0.GetDouble(1));
    frc::SmartDashboard::PutNumber("lidarDistance", lidarDistance.GetDouble(0));


    #ifdef PID_DEBUG
      UpperShooterPIDConfig.Debug("Upper Shooter PID Control");
    #endif

    //Run Gyro readings when the robot starts
    Gyro();
    frc::SmartDashboard::PutNumber("gyro angle", gyro_yawangledegrees);
}


/******************************************************************************
 * Function:     AutonomousInit
 *S
 * Description:  Function called at init while in autonomous.  This is where we
 *               should zero out anything that we need to before autonomous mode.
 ******************************************************************************/
void Robot::AutonomousInit()
  {
      int index;
      V_RobotInit = true;
      // visionInit(vision0, ledLight, inst);
      
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

      originalPosition = targetYaw0.GetDouble(0);
      vision1->PutBoolean("driverMode", true);
      inst.Flush();
      GyroZero();
  }


/******************************************************************************
 * Function:     AutonomousPeriodic
 *
 * Description:  Function called periodically in autonomous.  This is where we
 *               should place our primary autonomous control code.
 ******************************************************************************/
void Robot::AutonomousPeriodic()
  {   
      int theCoolerInteger;

      #pragma badauto
      // switch(theCoolerInteger)
      // {
      //     //1: Back up and shoot at the same time (5ft)
      //     case 1:
      //       if(autonComplete[3] == false)
      //       {
      //           if(AutoMove(distanceTarget, 457.0476) == false)
      //           {
      //             V_FWD = .5;
      //           }
      //           else
      //           {
      //             V_FWD = 0;
      //             autonComplete[1] = true;
      //           }

      //           V_ShooterSpeedDesired[E_TopShooter] = AutoShoot(targetYaw0, distanceTarget, 0);
      //           V_ShooterSpeedDesired[E_BottomShooter] = AutoShoot(targetYaw0, distanceTarget, 1);

      //           if (beamFullyCharged == true && beamCount < 3)
      //           {
      //             m_belt.Set(ControlMode::PercentOutput, 1);
      //           }
      //           else
      //           {
      //             m_belt.Set(ControlMode::PercentOutput, 0);
      //           }
                
      //           if (beamCount == 3)
      //           {
      //             autonComplete[2] = true;
      //           }

      //           if(autonComplete[1] && autonComplete[2])
      //           {
      //             autonComplete[3] = true;
      //           }
      //       }
      //       break;
          
      //     //Rotates then shoots, then rotates back to 0 position, then moves forward
      //     case 2:
      //       if(autonComplete[3] == false)
      //       {
      //         if(autonComplete[1] == false)
      //         {
      //           // V_RCW = AutoTarget(targetYaw0.GetDouble(0), distanceTarget, desiredVisionAngle0);

      //           V_ShooterSpeedDesired[E_TopShooter] = AutoShoot(targetYaw0, distanceTarget, 0);
      //           V_ShooterSpeedDesired[E_BottomShooter] = AutoShoot(targetYaw0, distanceTarget, 1);
                
      //           if(V_RCW == 0)
      //           {
      //             if(beamFullyCharged == true && beamCount < 3)
      //             {
      //               m_belt.Set(ControlMode::PercentOutput, 1);
      //             }
      //             else
      //             {
      //               m_belt.Set(ControlMode::PercentOutput, 0);
      //             }
                    
      //             if(beamCount == 3)
      //             {
      //               autonComplete[1] = true;
      //             }
      //           }
      //         }

      //         if (autonComplete[1])
      //         {
      //           if(abs(originalPosition) > 0)
      //           {
      //             // V_RCW = AutoTarget(-originalPosition, distanceTarget, desiredVisionAngle0);
      //           }
      //           else
      //           {
      //             autonComplete[2] = true;
      //           }
      //         }
              
      //         if(autonComplete[2])
      //         {
      //           if(AutoMove(distanceTarget, 457.0476) == false)
      //           {
      //             V_FWD = .5;
      //           }
      //           else
      //           {
      //             V_FWD = 0;
      //             autonComplete[3] = true;
      //           }
      //         }
      //       }
      //       break;
      // }
      #pragma endregion

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

      double timeleft = frc::DriverStation::GetInstance().GetMatchTime();
      double driveforward = 0;

      if(timeleft > 8)
      {
        V_ShooterSpeedDesiredFinalUpper = -1275;
        V_ShooterSpeedDesiredFinalLower = -1400;
      }
      else
      {
        V_ShooterSpeedDesiredFinalUpper = 0;
        V_ShooterSpeedDesiredFinalLower = 0;
      }
      V_ShooterSpeedDesired[E_TopShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_TopShooter], 50);
      V_ShooterSpeedDesired[E_BottomShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_BottomShooter], 50);

      if(timeleft < 12 && timeleft > 8)
      {
        m_elevateDaBalls.Set(ControlMode::PercentOutput, 0.69);
      }
      else
      {
        m_elevateDaBalls.Set(ControlMode::PercentOutput, 0);
      }

      if(timeleft < 8 && timeleft > 4)
      {
        driveforward = (-0.69 - 0.16);
      }

      DriveControlMain(driveforward,
                  0,
                  0,
                  c_joyStick.GetRawAxis(3),
                  c_joyStick.GetRawButton(1),
                  c_joyStick.GetRawButton(3),
                  c_joyStick.GetRawButton(4),
                  c_joyStick.GetRawButton(5),
                  gyro_yawangledegrees,
                  gyro_yawanglerad,
                  targetYaw0.GetDouble(0),
                  &V_WheelAngleFwd[0],
                  &V_WheelAngleRev[0],
                  &V_WS[0],
                  &V_WA[0],
                  &V_RobotInit,
                  V_AutoTargetState,
                  V_AutoTargetAngle);
        
        T_RobotCorner index; 

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

    m_topShooterpid.SetReference(V_ShooterSpeedDesired[E_TopShooter], rev::ControlType::kVelocity);
    m_bottomShooterpid.SetReference(V_ShooterSpeedDesired[E_BottomShooter], rev::ControlType::kVelocity);

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
      V_AutoShootEnable = false;

  BallsShot = 0;

  GyroZero();
}


/******************************************************************************
 * Function:     TeleopPeriodic
 *
 * Description:  Primary function called when in teleop mode.
 ******************************************************************************/
void Robot::TeleopPeriodic()
  {
  T_RobotCorner         index;
  double                L_FortuneMotor;
  // T_WheelOfFortuneColor L_Color;

  //L_Color = ColorSensor(false);
  L_FortuneMotor = WheelOfFortune (//L_Color,
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

  double L_JoyStick1Axis1Y = DesiredSpeed(c_joyStick.GetRawAxis(1));
  double L_JoyStick1Axis1X = DesiredSpeed(c_joyStick.GetRawAxis(0));
  double L_JoyStick1Axis2X = DesiredSpeed(c_joyStick.GetRawAxis(4));

  DriveControlMain(L_JoyStick1Axis1Y,
                   L_JoyStick1Axis1X,
                   L_JoyStick1Axis2X,
                   c_joyStick.GetRawAxis(3),
                   c_joyStick.GetRawButton(1),
                   c_joyStick.GetRawButton(3),
                   c_joyStick.GetRawButton(4),
                   c_joyStick.GetRawButton(5),
                   gyro_yawangledegrees,
                   gyro_yawanglerad,
                   targetYaw0.GetDouble(0),
                   &V_WheelAngleFwd[0],
                   &V_WheelAngleRev[0],
                   &V_WS[0],
                   &V_WA[0],
                   &V_RobotInit,
                   V_AutoTargetState,
                   V_AutoTargetAngle);

  //PDP top shooter port 13
  //PDP bottom shooter port 12
//  PDP_Current_UpperShooter = PDP.GetCurrent(13);
//  PDP_Current_LowerShooter = PDP.GetCurrent(12);
//  if(abs(PDP_Current_LowerShooter - PDP_Current_LowerShooter_last) > 2 || abs(PDP_Current_UpperShooter - PDP_Current_UpperShooter_last) > 2)
//  {
//    BallsShot += 1;
//  }
//  PDP_Current_UpperShooter_last = PDP_Current_UpperShooter;
//  PDP_Current_LowerShooter_last = PDP_Current_LowerShooter;


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

    frc::SmartDashboard::PutNumber("Gyro Angle Deg", gyro_yawangledegrees);
    // frc::SmartDashboard::PutNumber("Gyro Angle Rad", gyro_yawanglerad);
    // frc::SmartDashboard::PutNumber("ROLL OVER RAD", gyro_rolloverrad);

    // frc::SmartDashboard::PutNumber("Front Left Wheel Velocity",(V_WheelVelocity[E_FrontLeft]));
    // frc::SmartDashboard::PutNumber("Front Right Wheel Velocity",(V_WheelVelocity[E_FrontRight]));
    // frc::SmartDashboard::PutNumber("Rear Left Wheel Velocity",(V_WheelVelocity[E_RearLeft]));
    // frc::SmartDashboard::PutNumber("Rear Right Wheel Velocity",(V_WheelVelocity[E_RearRight]));

    // frc::SmartDashboard::PutNumber("STR", V_STR);
    // frc::SmartDashboard::PutNumber("FWD", V_FWD);
    // frc::SmartDashboard::PutNumber("RCW", V_RCW);

    // frc::SmartDashboard::PutNumber("Wheel angle FR", V_WheelAngleArb[E_FrontRight]);
    // frc::SmartDashboard::PutNumber("Wheel angle FL", V_WheelAngleArb[E_FrontLeft]);
    // frc::SmartDashboard::PutNumber("Wheel angle RR", V_WheelAngleArb[E_RearRight]);
    // frc::SmartDashboard::PutNumber("Wheel angle RL", V_WheelAngleArb[E_RearLeft]);

    // frc::SmartDashboard::PutNumber("WS_FR", V_WS[E_FrontRight]);
    // frc::SmartDashboard::PutNumber("WS_FL", V_WS[E_FrontLeft]);
    // frc::SmartDashboard::PutNumber("WS_RL", V_WS[E_RearLeft]);
    // frc::SmartDashboard::PutNumber("WS_RR", V_WS[E_RearRight]);

    // frc::SmartDashboard::PutNumber("WA_FR", V_WA[E_FrontRight]);
    // frc::SmartDashboard::PutNumber("WA_FL", V_WA[E_FrontLeft]);
    // frc::SmartDashboard::PutNumber("WA_RL", V_WA[E_RearLeft]);
    // frc::SmartDashboard::PutNumber("WA_RR", V_WA[E_RearRight]);

    // frc::SmartDashboard::PutBoolean("RobotInit",  V_RobotInit);

    //Shooter mech
    frc::SmartDashboard::PutNumber("Top Speed", V_ShooterSpeedCurr[E_TopShooter]);
    frc::SmartDashboard::PutNumber("Bottom Speed", V_ShooterSpeedCurr[E_BottomShooter]);

    // SpeedRecommend = (distanceTarget * sqrt(-9.807 / (2 * cos(35 * deg2rad) * cos(35 * deg2rad) * (1.56845 - (distanceTarget * tan(35 * deg2rad))))));
    // frc::SmartDashboard::PutNumber("Recommended Speed", SpeedRecommend);

    //NOTE  Zero Gyro
    if(c_joyStick.GetRawButton(7))
    {
      GyroZero();
    }


    //Shooter mech controls
    // int shooterSpeedSlowDown;
    // T_RoboShooter dex;
    
    //NOTE: Shooting code slowdown
    // if(c_joyStick2.GetRawButton(8))
    // {   
    //   if(shooterSpeedSlowDown < 200)
    //   {
    //     for (dex = E_TopShooter;
    //          dex < E_RoboShooter;
    //          dex = T_RoboShooter(int(dex) + 1))
    //     {
    //         V_ShooterSpeedDesired[dex] *= 0.99;
    //         shooterSpeedSlowDown++;
    //     }
    //   }       
    //   else if(shooterSpeedSlowDown >= 200)
    //   {
    //     V_ShooterSpeedDesired[E_TopShooter] = 0;
    //     V_ShooterSpeedDesired[E_BottomShooter] = 0;
    //   }
    // }
    // if(c_joyStick2.GetRawButton(8))
    // {

    // }
  #ifdef COMP
   if ((c_joyStick2.GetPOV() == 90))
   {
     V_AutoShootEnable = false;
     V_ShooterSpeedDesiredFinalUpper = 0;
     V_ShooterSpeedDesiredFinalLower = 0;
   }
    
    if ((c_joyStick2.GetPOV() == 180) || (c_joyStick2.GetPOV() == 270) || (c_joyStick2.GetPOV() == 0) || (V_AutoShootEnable == true))
    {
      // V_ShooterSpeedDesired[E_TopShooter] = -3000;
      // V_ShooterSpeedDesired[E_BottomShooter] = -3200;
      if ((c_joyStick2.GetPOV() == 180))
      {
       V_ShooterSpeedDesiredFinalUpper = (-1312.5); //-1312.5
       V_ShooterSpeedDesiredFinalLower = (-1400 * .8); //-1400
      }
      else if ((c_joyStick2.GetPOV() == 270))
      {
       V_ShooterSpeedDesiredFinalUpper = -2350;
       V_ShooterSpeedDesiredFinalLower = -3325;
      }
      else if ((c_joyStick2.GetPOV() == 0))
      {
       V_ShooterSpeedDesiredFinalUpper = -200;
       V_ShooterSpeedDesiredFinalLower = -200;
      }

      V_AutoShootEnable = true;
      V_ShooterSpeedDesired[E_TopShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_TopShooter], 50);
      V_ShooterSpeedDesired[E_BottomShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_BottomShooter], 50);
    }
    else if(fabs(c_joyStick2.GetRawAxis(5)) > .05 || fabs(c_joyStick2.GetRawAxis(1)) > .05)
    {
      V_ShooterSpeedDesired[E_TopShooter] = c_joyStick2.GetRawAxis(1);
      V_ShooterSpeedDesired[E_BottomShooter] = c_joyStick2.GetRawAxis(5);
    } else {
      V_ShooterSpeedDesired[E_TopShooter] = 0;
      V_ShooterSpeedDesired[E_BottomShooter] = 0;
    }
#endif
#ifdef TEST
    V_ShooterSpeedDesiredFinalUpper    = frc::SmartDashboard::GetNumber("Speed Desired Top", 0);
    V_ShooterSpeedDesiredFinalLower = frc::SmartDashboard::GetNumber("Speed Desired Bottom", 0);
    V_ShooterSpeedDesired[E_TopShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_TopShooter], 40);
    V_ShooterSpeedDesired[E_BottomShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_BottomShooter], 40);
#endif
    
    // // else
    // // {
    // V_ShooterSpeedDesiredFinalUpper    = frc::SmartDashboard::GetNumber("Speed Desired Top", 0);
    // V_ShooterSpeedDesiredFinalLower = frc::SmartDashboard::GetNumber("Speed Desired Bottom", 0);
    // // }
    // V_ShooterSpeedDesired[E_TopShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_TopShooter], 40);
    // V_ShooterSpeedDesired[E_BottomShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_BottomShooter], 40);
    //Shooter mech
    #ifdef TEST
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

    m_topShooterpid.SetP(upper_P_Gx);
    m_topShooterpid.SetI(upper_I_Gx);
    m_topShooterpid.SetD(upper_D_Gx);
    m_topShooterpid.SetIZone(upper_I_Zone);
    m_topShooterpid.SetFF(upper_FF);
    m_topShooterpid.SetOutputRange(upper_Min, upper_Max);

    m_bottomShooterpid.SetP(lower_P_Gx);
    m_bottomShooterpid.SetI(lower_I_Gx);
    m_bottomShooterpid.SetD(lower_D_Gx);
    m_bottomShooterpid.SetIZone(lower_I_Zone);
    m_bottomShooterpid.SetFF(lower_FF);
    m_bottomShooterpid.SetOutputRange(lower_Min, lower_Max);
    #endif





    frc::SmartDashboard::PutNumber("Postion", m_encoderLift.GetPosition());

    if(c_joyStick2.GetRawAxis(3) > 0.1)
    {
      if(m_encoderLift.GetPosition() > -480)
      {
        m_liftMotor.Set(c_joyStick2.GetRawAxis(3) * -1.0);
      }
      else
      {
        m_liftMotor.Set(0);
      }
    }
    else if (c_joyStick2.GetRawAxis(2) > 0.1)
    {
      if(m_encoderLift.GetPosition() < -30)
      {
        m_liftMotor.Set(c_joyStick2.GetRawAxis(2) * 1.0);
      }
      else
      {
        m_liftMotor.Set(0);
      }
    }
    else if(c_joyStick2.GetRawButton(6))
    {
      m_liftMotor.Set(0.025);
    }
    else
    {
      m_liftMotor.Set(0);
    }

    frc::SmartDashboard::PutNumber("Upper Velocity", m_encoderTopShooter.GetVelocity());
    frc::SmartDashboard::PutNumber("Lower Velocity", m_encoderBottomShooter.GetVelocity());

#ifdef COMP
if (V_AutoShootEnable == true)
{
    m_topShooterpid.SetReference(V_ShooterSpeedDesired[E_TopShooter], rev::ControlType::kVelocity);
    m_bottomShooterpid.SetReference(V_ShooterSpeedDesired[E_BottomShooter], rev::ControlType::kVelocity);
}
else 
{
   m_topShooterMotor.Set(V_ShooterSpeedDesired[E_TopShooter]);
    m_bottomShooterMotor.Set(V_ShooterSpeedDesired[E_BottomShooter]);
}
#endif
#ifdef TEST
    m_topShooterpid.SetReference(V_ShooterSpeedDesired[E_TopShooter], rev::ControlType::kVelocity);
    m_bottomShooterpid.SetReference(V_ShooterSpeedDesired[E_BottomShooter], rev::ControlType::kVelocity);
#endif



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
//    if(c_joyStick2.GetRawButton(2))
//    {
//      intake.Set(frc::DoubleSolenoid::Value::kForward);
//      m_intake.Set(ControlMode::PercentOutput, 0.5);
//    }
//    else
//    {
//      intake.Set(frc::DoubleSolenoid::Value::kOff);
//      m_intake.Set(ControlMode::PercentOutput, 0);
//    }

    bool active = ir_sensor.Get();

    if(c_joyStick2.GetRawButton(1))
    {
      if(active)
      {
        m_conveyDaBalls.Set(ControlMode::PercentOutput, -1);
      }
      m_elevateDaBalls.Set(ControlMode::PercentOutput, 0.4);
    }
    else if(c_joyStick2.GetRawButton(2))
    {
      if(active)
      {
        m_elevateDaBalls.Set(ControlMode::PercentOutput, -0.420);
      }
      m_conveyDaBalls.Set(ControlMode::PercentOutput, 0.420);
    }
    else
    {
      m_conveyDaBalls.Set(ControlMode::PercentOutput, 0);
      m_elevateDaBalls.Set(ControlMode::PercentOutput, 0);
    }

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
