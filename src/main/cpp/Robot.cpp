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


double V_WheelAngleCmnd[E_RobotCornerSz];
double V_WheelAngleError[E_RobotCornerSz];
double V_WheelAngleIntegral[E_RobotCornerSz];
double V_WheelSpeedCmnd[E_RobotCornerSz];
double V_WheelSpeedError[E_RobotCornerSz];
double V_WheelSpeedIntergral[E_RobotCornerSz];

bool   V_RobotInit;

double V_WheelAngleCase[E_RobotCornerSz];  // For trouble shooting where the angle is coming from

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

double desiredAngle;
double rotateDeBounce;
double rotateErrorCalc;
double rotateErrorIntegral;
bool rotateMode;
double P_Gx;
double I_Gx;
double P_UL;
double P_LL; 
double I_UL;
double I_LL;
double SpeedRecommend;

/******************************************************************************
 * Function:     visionInit
 *
 * Description:  This function sets up camera and lights.
 ******************************************************************************/
void visionInit(std::shared_ptr<NetworkTable> ntTable0,
                std::shared_ptr<NetworkTable> ntTable1)
{
    ntTable1->PutNumber("ledControl", 1);
    ntTable0->PutBoolean("driverMode", false);
    inst.Flush();
}

/******************************************************************************
 * Function:     visionOff
 *
 * Description:  This function turns on driver mode and turns off lights.
 ******************************************************************************/
void visionOff(std::shared_ptr<NetworkTable> ntTable0,
                std::shared_ptr<NetworkTable> ntTable1)
{
    ntTable1->PutNumber("ledControl", 5);
    ntTable0->PutBoolean("driverMode", true);
    inst.Flush();
    visionRequest = false;
    visionStart[1] = false; visionStart[2] = false;
    activeVisionAngle0 = false;
    activeVisionDistance0 = false;
}
    
/******************************************************************************
 * Function:     visionRun
 *
 * Description:  This function toggles vision loop.
 ******************************************************************************/
void visionRun(nt::NetworkTableEntry ntEntry,
              double ntDistance,
              char targetChoose,
              std::shared_ptr<NetworkTable> ntTable0,
              std::shared_ptr<NetworkTable> ntTable1)
{
      switch (targetChoose)
      {
          case high:
            if (abs(ntEntry.GetDouble(0)) > 1)
            {
                autonChoose = rotate;
            }
            if (abs(ntEntry.GetDouble(0)) < 1)
            {
                autonChoose = complete;
            }
            break;

          case ball:
            if (abs(ntEntry.GetDouble(0)) > 5)
            {
                autonChoose = strafe;
            }
            else if (abs(ntEntry.GetDouble(0)) > 1)
            {
                activeVisionDistance0 = false;
                autonChoose = rotate;
            }
            if (abs(ntEntry.GetDouble(0)) < 1)
            {
                autonChoose = complete;
            }
            break;
      }
      
      switch (autonChoose)
      {
          case strafe:
            if (activeVisionDistance0 == false)
            {
                desiredVisionDistance0 = floor(ntDistance);
                activeVisionDistance0  = true;
            }
            if (visionRequest == true)
            {
                visionRequest = false;
                activeVisionDistance0 = false;
            }
            break;
          
          case rotate:
            if (activeVisionDistance0 == false)
            {
                desiredVisionAngle0 = (0.75 * ntEntry.GetDouble(0));
                activeVisionAngle0  = true;
            }
            if (visionRequest == true)
            {
                visionRequest = false;
                activeVisionDistance0 = false;
            }
            break;
          
          case complete:
            visionOff(ntTable0, ntTable1);
      }
}

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

  return (L_CriteriaMet);
  }

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
    inst.StartClient("10.55.61.50");
    inst.StartDSClient();

    vision0  = inst.GetTable("chameleon-vision/LED Camera 0");
    vision1  = inst.GetTable("chameleon-vision/LifeCam 1");
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

    V_ShooterSpeedError[E_TopShooter] = 0;
    V_ShooterSpeedError[E_BottomShooter] = 0;
    V_ShooterSpeedIntegral[E_TopShooter] = 0;
    V_ShooterSpeedIntegral[E_BottomShooter] = 0;
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
  else if (c_joyStick.GetRawButton(1)) {
    rotateMode = true;
    desiredAngle = 112.5;
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

  if (rotateMode == true) {
      V_RCW = Control_PID(desiredAngle,
                                            gyro_yawangledegrees,
                                            &rotateErrorCalc,
                                            &rotateErrorIntegral,
                                            0.08,   // P Gx 0.0038 0.005
                                            0.0007, // I Gx 0.0029  0.001
                                            0, // D Gx
                                            0.9, // P UL
                                            -0.9, // P LL
                                            0.5, // I UL
                                            -0.5, // I LL
                                            0.2, // D UL
                                            -0.2, // D LL
                                            1.0, // Max upper
                                            -1.0);

    // if (errorCalc > 0) {
    // V_RCW = 0.8;
    
    // }
    // else if (errorCalc < 0) {
    // V_RCW = -0.8;
    // }
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




    frc::SmartDashboard::PutBoolean("rotateMode",rotateMode);
    frc::SmartDashboard::PutNumber("errorCalc",errorCalc);
    frc::SmartDashboard::PutNumber("desiredAngle",desiredAngle);

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

    if(c_joyStick2.GetRawButton(4))
    {
      m_intake.Set(ControlMode::PercentOutput, 0.9);
    } else {
      m_intake.Set(ControlMode::PercentOutput, 0);
    }
    
        //Shooter mech
    T_RoboShooter dex;
    V_ShooterSpeedCurr[E_TopShooter]    = (m_encoderTopShooter.GetVelocity()    * shooterWheelRotation) * 0.3191858136047229930278045677412;
    V_ShooterSpeedCurr[E_BottomShooter] = (m_encoderBottomShooter.GetVelocity() * shooterWheelRotation) * 0.2393893602035422447708534258059;

    frc::SmartDashboard::PutNumber("Top Speed", V_ShooterSpeedCurr[E_TopShooter]);
    frc::SmartDashboard::PutNumber("Bottom Speed", V_ShooterSpeedCurr[E_BottomShooter]);

    SpeedRecommend = (distanceTarget * sqrt(-9.807 / (2 * cos(35 * deg2rad) * cos(35 * deg2rad) * (1.56845 - (distanceTarget * tan(35 * deg2rad))))));
    frc::SmartDashboard::PutNumber("Recommended Speed", SpeedRecommend);

    
    //Shooter mech controls
    // V_ShooterSpeedDesired[E_TopShooter]    = c_joyStick2.GetRawAxis(1) * 10000;
    // V_ShooterSpeedDesired[E_BottomShooter] = c_joyStick2.GetRawAxis(5) * 10000;

    V_ShooterSpeedDesired[E_TopShooter]    = frc::SmartDashboard::GetNumber("Speed Desired Top", 0);
    V_ShooterSpeedDesired[E_BottomShooter] = frc::SmartDashboard::GetNumber("Speed Desired Bottom", 0);

    
    // if (c_joyStick2.GetRawButton(1) == true)
    // {
    //     V_ShooterSpeedCmnd[E_TopShooter] = -.75;
    //     V_ShooterSpeedCmnd[E_BottomShooter] = -1.0;
    // }

    // if (c_joyStick2.GetRawButton(2) == true)
    // {
    //     V_ShooterRequest[1] = true;
    // }

    // if (c_joyStick2.GetRawButton(3) == true)
    // {
    //     V_ShooterRequest[1] = false; V_ShooterRequest[2] = false;
    // }


    //Shooter mech logic
    P_Gx = frc::SmartDashboard::GetNumber("P_Gx", 0);
    I_Gx = frc::SmartDashboard::GetNumber("I_Gx", 0);
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

    

    // m_topShooterMotor.Set(V_ShooterSpeedCmnd[E_TopShooter]);
    // m_bottomShooterMotor.Set(V_ShooterSpeedCmnd[E_BottomShooter]);  

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

    frc::SmartDashboard::PutNumber("Upper Velocity", m_encoderTopShooter.GetVelocity());
    frc::SmartDashboard::PutNumber("Lower Velocity", m_encoderBottomShooter.GetVelocity());

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

    m_frontLeftSteerMotor.Set (V_WheelAngleCmnd[E_FrontLeft] * (-1));
    m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight] * (-1));
    m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft] * (-1));
    m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight] * (-1));

    // m_frontLeftSteerMotor.Set(0);
    // m_frontRightSteerMotor.Set(0);
    // m_rearLeftSteerMotor.Set(0);
    // m_rearRightSteerMotor.Set(0);

    /*
      Finds distance from robot to specified target.
      Numerator depends upon camera height relative to target for target distance,
      and camera height relative to ground for ball distance.
      Make sure it's in meters.
    */
    distanceTarget     = 157.8 / tan((targetPitch0.GetDouble(0)) * (deg2rad));
    distanceBall       = 47  / tan((targetPitch1.GetDouble(0)) * (-deg2rad));

frc::SmartDashboard::PutNumber("distanceTarget", distanceTarget);
    //Finds robot's distance from target's center view.
    distanceFromTargetCenter = distanceTarget * sin((90 - targetYaw0.GetDouble(0)) * deg2rad);
    distanceFromBallCenter   = distanceBall   * sin((90 - targetYaw1.GetDouble(0)) * deg2rad); 

    //Toggle for target adjust
    if(c_joyStick.GetRawButton(1) == true)
     {
         visionInit(vision0, ledLight);
         visionStart[1] = true;
     }

     if(c_joyStick.GetRawButton(2) == true)
     {
         visionInit(vision0, ledLight);
         visionStart[2] = true;
     }

     if(c_joyStick.GetRawButton(3) == true)
     {
        visionRequest = true;
     }

     if(c_joyStick.GetRawButton(4) == true)
     {
        visionStart[1] = false; visionStart[2] = false;
     }

     if(visionStart[1] == true && visionStart[2] == false)
     {
        visionRun(targetYaw0, distanceFromTargetCenter, high, vision0, ledLight);
     }

    if(visionStart[2] == true && visionStart[1] == false)
    {
        visionRun(targetYaw0, -distanceFromBallCenter, ball, vision0, ledLight);
    }

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
