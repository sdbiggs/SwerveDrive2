/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
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

double V_WS[E_RobotCornerSz];
double V_WA[E_RobotCornerSz];
double V_WA_Prev[E_RobotCornerSz];
double V_WA_Loopcount[E_RobotCornerSz];
double V_WA_Final[E_RobotCornerSz];

double V_WheelRPM[E_RobotCornerSz];
//double V_WheelAngle[E_RobotCornerSz];
//double V_WheelRelativeAngleRawOffset[E_RobotCornerSz];
double V_WheelAngleCmnd[E_RobotCornerSz];
double V_WheelSpeedCmnd[E_RobotCornerSz];
double V_WheelSpeedIntergral[E_RobotCornerSz];
double V_WheelRpmCmnd[E_RobotCornerSz];
//double V_WheelVelocity[E_RobotCornerSz];
double V_WheelAngleError[E_RobotCornerSz];
double V_WheelAngleIntegral[E_RobotCornerSz];
//double V_DesiredWheelAngle[E_RobotCornerSz];
//double V_DesiredWheelSpeed[E_RobotCornerSz];
bool   V_RobotInit;
bool   V_ModeTransition;
int    V_Mode;
bool   V_WheelSpeedDelay;
double V_WheelSpeedError[E_RobotCornerSz];
double V_WheelAngleCase[E_RobotCornerSz];
double V_FWD;
double V_STR;
double V_RCW;
double V_GAIN;

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
nt::NetworkTableEntry driverMode;

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
    //Wait(.5);
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

void Robot::RobotInit() {
//    m_leftLeadMotor.RestoreFactoryDefaults();
//    m_rightLeadMotor.RestoreFactoryDefaults();
//    m_leftFollowMotor.RestoreFactoryDefaults();
//    m_rightFollowMotor.RestoreFactoryDefaults();

    m_frontLeftSteerMotor.RestoreFactoryDefaults();
    m_frontLeftDriveMotor.RestoreFactoryDefaults();
    m_frontRightSteerMotor.RestoreFactoryDefaults();
    m_frontRightDriveMotor.RestoreFactoryDefaults();
    m_rearLeftSteerMotor.RestoreFactoryDefaults();
    m_rearLeftDriveMotor.RestoreFactoryDefaults();
    m_rearRightSteerMotor.RestoreFactoryDefaults();
    m_rearRightDriveMotor.RestoreFactoryDefaults();

    V_RobotInit = false;
    V_WheelSpeedDelay = false;

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
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit(){
  V_RobotInit = true;
  V_WheelSpeedDelay = false;
  V_ModeTransition = false;
  V_Mode = 0;

    m_frontLeftSteerMotor.RestoreFactoryDefaults();
    m_frontLeftDriveMotor.RestoreFactoryDefaults();
    m_frontRightSteerMotor.RestoreFactoryDefaults();
    m_frontRightDriveMotor.RestoreFactoryDefaults();
    m_rearLeftSteerMotor.RestoreFactoryDefaults();
    m_rearLeftDriveMotor.RestoreFactoryDefaults();
    m_rearRightSteerMotor.RestoreFactoryDefaults();
    m_rearRightDriveMotor.RestoreFactoryDefaults();

  for (int index = E_FrontLeft;
         index < E_RobotCornerSz;
         index = T_RobotCorner(int(index) + 1))
      {
        V_WS[index] = 0;
        V_WA[index] = 0;
        V_DesiredWheelAngle[index] = 0;
        V_WheelRelativeAngleRawOffset[index] = 0;
        V_WheelAngle[index] = 0;
        V_WheelAnglePrev[index] = 0;
        V_WheelAngleLoop[index] = 0;
        V_WheelAngleRaw[index] = 0;
        V_WheelAngleError[index] = 0;
        V_WheelAngleIntegral[index] = 0;
        V_WheelVelocity[index] = 0;
        V_WheelSpeedError[index] = 0;
        V_WheelSpeedIntergral[index] = 0; 
      }
      V_STR = 0;
      V_FWD = 0;
      gyro_yawangledegrees = 0;
      gyro_yawanglerad = 0;

  GyroTeleInit();
}

void Robot::TeleopPeriodic() {
  T_RobotCorner index = E_FrontLeft;

  ColorSensor(false);
  Gyro(); 
  bool L_WheelSpeedDelay = false;

  Read_Encoders(V_RobotInit, 
  a_encoderFrontLeftSteer.GetVoltage(), a_encoderFrontRightSteer.GetVoltage(), a_encoderRearLeftSteer.GetVoltage(), a_encoderRearRightSteer.GetVoltage(), 
  m_encoderFrontLeftSteer,m_encoderFrontRightSteer, m_encoderRearLeftSteer, m_encoderRearRightSteer, 
  m_encoderFrontLeftDrive, m_encoderFrontRightDrive,m_encoderRearLeftDrive, m_encoderRearRightDrive);

  V_FWD = c_joyStick.GetRawAxis(1) * -1;
  V_STR = c_joyStick.GetRawAxis(0);
  V_RCW = c_joyStick.GetRawAxis(4);
  V_GAIN = c_joyStick.GetRawAxis(3);

  SwerveDriveWheelOutput(V_FWD,V_STR,V_RCW,V_GAIN,&V_WheelAngle[0], &V_WA[0], &V_WS[0]);
  
  if(c_joyStick.GetRawButtonPressed(2))
  {
    rotatemode = true;
  }

  if(rotatemode == true)
  {
    for (index = E_FrontLeft; index< E_RobotCornerSz; index = T_RobotCorner(int(index) + 1)) {
      V_DesiredWheelAngle[E_RobotCornerSz] = 90;
    }
  }


  frc::SmartDashboard::PutNumber("FL Case",(V_WheelAngleCase[E_FrontLeft]));
  frc::SmartDashboard::PutNumber("FR Case",(V_WheelAngleCase[E_FrontRight]));
  frc::SmartDashboard::PutNumber("RL Case",(V_WheelAngleCase[E_RearLeft]));
  frc::SmartDashboard::PutNumber("RR Case",(V_WheelAngleCase[E_RearRight]));
  

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

  frc::SmartDashboard::PutNumber("STR", V_STR);
  frc::SmartDashboard::PutNumber("FWD", V_FWD);
  frc::SmartDashboard::PutNumber("RCW", V_RCW);
    
    if (V_RobotInit == true)
      {
      V_RobotInit = false;
      for (index = E_FrontLeft;
           index < E_RobotCornerSz;
           index = T_RobotCorner(int(index) + 1))
        {
        V_WS[index] = 0;
        V_WA[index] = 0;
        V_WA_Final[index] = 0;
        if (fabs(V_WheelAngle[index]) > 1.2)
          {
          V_RobotInit = true;
          }
        }
      }
      

    for (index = E_FrontLeft;
         index < E_RobotCornerSz;
         index = T_RobotCorner(int(index) + 1))
      {
      V_WheelAngleCmnd[index] =  Control_PID( V_WA[index],
                                              V_WheelAngle[index],
                                             &V_WheelAngleError[index],
                                             &V_WheelAngleIntegral[index],
                                              0.0045, // P Gx 0.0045
                                              0.0001, // I Gx 0001
                                              0.0, // D Gx
                                              0.4, // P UL
                                             -0.4, // P LL
                                              0.12, // I UL
                                             -0.12, // I LL
                                              0.0, // D UL
                                             -0.0, // D LL
                                              0.9, // Max upper
                                             -0.9); // Max lower

      V_WheelSpeedCmnd[index] = Control_PID(V_WS[index],
                                      V_WheelVelocity[index], 
                                      &V_WheelSpeedError[index], 
                                      &V_WheelSpeedIntergral[index], 
                                      0.0038, // P Gx
                                      0.0029, // I Gx
                                      0.0, // D Gx
                                      0.9, // P UL
                                      -0.9, // P LL
                                      0.9, // I UL
                                      -0.9, // I LL
                                      0.0, // D UL
                                      -0.0, // D LL
                                      1.0, // Max upper
                                      -1.0);
      }
    //Ws1: fr, Ws2: fl, ws3: rl, ws4: rr

    frc::SmartDashboard::PutNumber("Drive Intergral Front Left", V_WheelSpeedIntergral[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("Drive Intergral Front Right", V_WheelSpeedIntergral[E_FrontRight]);
    frc::SmartDashboard::PutNumber("Drive Intergral Rear Left", V_WheelSpeedIntergral[E_RearLeft]);
    frc::SmartDashboard::PutNumber("Drive Intergral Rear Right", V_WheelSpeedIntergral[E_RearRight]);

    frc::SmartDashboard::PutNumber("Wheel angle FR", V_WheelAngle[E_FrontRight]);
    frc::SmartDashboard::PutNumber("Wheel angle FL", V_WheelAngle[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("Wheel angle RR", V_WheelAngle[E_RearRight]);
    frc::SmartDashboard::PutNumber("Wheel angle RL", V_WheelAngle[E_RearLeft]);

    frc::SmartDashboard::PutNumber("WS_FR", V_WS[E_FrontRight]);
    frc::SmartDashboard::PutNumber("WS_FL", V_WS[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("WS_RL", V_WS[E_RearLeft]);
    frc::SmartDashboard::PutNumber("WS_RR", V_WS[E_RearRight]);

    frc::SmartDashboard::PutNumber("WA_FR", V_WA[E_FrontRight]);
    frc::SmartDashboard::PutNumber("WA_FL", V_WA[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("WA_RL", V_WA[E_RearLeft]);
    frc::SmartDashboard::PutNumber("WA_RR", V_WA[E_RearRight]);

    
    frc::SmartDashboard::PutNumber("Wheel angle integral FR", V_WheelAngleIntegral[E_FrontRight]);
    frc::SmartDashboard::PutNumber("Wheel angle integral FL", V_WheelAngleIntegral[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("Wheel angle integral RR", V_WheelAngleIntegral[E_RearRight]);
    frc::SmartDashboard::PutNumber("Wheel angle integral RL", V_WheelAngleIntegral[E_RearLeft]);

  
    //Shooter mech
    T_RoboShooter dex;
    V_ShooterSpeedCurr[E_TopShooter]         = (m_encoderTopShooter.GetVelocity()    * shooterWheelRotation) * 0.3191858136047229930278045677412;
    V_ShooterSpeedCurr[E_BottomShooter]      = (m_encoderBottomShooter.GetVelocity() * shooterWheelRotation) * 0.2393893602035422447708534258059;
    
    //Shooter mech controls
    V_ShooterSpeedCmnd[E_TopShooter]         = c_joyStick2.GetRawAxis(1) * -1;
    V_ShooterSpeedCmnd[E_BottomShooter]      = c_joyStick2.GetRawAxis(5) * -1;
    
    if (c_joyStick2.GetRawButton(1) == true)
    {
        V_ShooterSpeedCmnd[dex] = 0;
    }

    if (c_joyStick2.GetRawButton(2) == true)
    {
        V_ShooterRequest[1] = true;
    }

    if (c_joyStick2.GetRawButton(3) == true)
    {
        V_ShooterRequest[1] = false; V_ShooterRequest[2] = false;
    }


    //Shooter mech logic
    for (dex = E_TopShooter;
         dex < E_RoboShooter;
         dex = T_RoboShooter(int(dex) + 1))
    {
        V_ShooterSpeedCmnd[dex] = Control_PID(V_ShooterSpeedDesired[dex],
                                              V_ShooterSpeedCurr[dex],
                                              &V_ShooterSpeedError[dex],
                                              &V_ShooterSpeedIntegral[dex],
                                              0.0069, // P Gx
                                              0.000069, // I Gx
                                              0.0, // D Gx
                                              0.069, // P UL
                                             -0.069, // P LL
                                              0.0069, // I UL
                                             -0.0069, // I LL
                                              0.0, // D UL
                                             -0.0, // D LL
                                              0.69, // Max upper
                                             -0.69); // Max lower
    }


    //Shooter mech w/ vision assist
    if (V_ShooterRequest[1] == true)
    {
        for (dex = E_TopShooter;
             dex < E_RoboShooter;
             dex = T_RoboShooter(int(dex) + 1))
        {
             V_ShooterSpeedDesired[dex] = (distanceTarget * sqrt(-9.807 / (2 * cos(C_PI / 4) * cos(C_PI / 4) * (1.56845 - (distanceTarget * tan(C_PI / 4))))));
             V_ShooterRequest[2] = true;
        }
    }

    if (V_ShooterRequest[2] == true)
    {
        //starrt loading balls
    }



    m_topShooterMotor.Set(V_ShooterSpeedCmnd[E_TopShooter]);
    m_bottomShooterMotor.Set(V_ShooterSpeedCmnd[E_BottomShooter]);    

    m_frontLeftDriveMotor.Set(V_WheelSpeedCmnd[E_FrontLeft]);
    m_frontRightDriveMotor.Set(V_WheelSpeedCmnd[E_FrontRight]);
    m_rearLeftDriveMotor.Set(V_WheelSpeedCmnd[E_RearLeft]);
    m_rearRightDriveMotor.Set(V_WheelSpeedCmnd[E_RearRight]);

     
    m_frontLeftSteerMotor.Set (V_WheelAngleCmnd[E_FrontLeft] * (-1));
    m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight] * (-1));
    m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft] * (-1));
    m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight] * (-1));

    /*
      Finds distance from robot to specified target.
      Numerator depends upon camera height relative to target for target distance,
      and camera height relative to ground for ball distance.
      Make sure it's in meters.
    */
    distanceTarget     = 169 / tan((targetPitch0.GetDouble(0)) * (deg2rad));
    distanceBall       = 47  / tan((targetPitch1.GetDouble(0)) * (-deg2rad));

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

    // m_frontLeftSteerMotor.Set(0);
    // m_frontRightSteerMotor.Set(0);
    // m_rearLeftSteerMotor.Set(0);
    // m_rearRightSteerMotor.Set(0);

   /* if (a_joyStick.GetRawButton(0) && driver_mode == false)
    {
      table->PutBoolean("driver_mode", true);
    }
    else if ((a_joyStick.GetRawButton(0) && driver_mode == true)
    {
      table->PutBoolean("driver_mode", false);
    }
    */
   
    //driverMode = vision->PutBoolean("driver_mode", true);

    // vision->GetBooleanArray()

    // /**
    //  * Open Smart Dashboard or Shuffleboard to see the color detected by the 
    //  * sensor.
    //  */
    //lw->Add
  
    //frc::SmartDashboard::PutNumberArray("Anglealjshdfkjshdfsdhfklsjdfh Desired Array", V_DesiredWheelAngle);

    frc::SmartDashboard::PutBoolean("Wheel Delay",  V_WheelSpeedDelay);
    frc::SmartDashboard::PutBoolean("RobotInit",  V_RobotInit);
    frc::SmartDashboard::PutBoolean("driver_mode", driverMode.GetBoolean(false));
   
    frc::SmartDashboard::PutNumber("Angle Desire Front Left", V_DesiredWheelAngle[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("Angle Desire Front Right", V_DesiredWheelAngle[E_FrontRight]);
    frc::SmartDashboard::PutNumber("Angle Desire Rear Left", V_DesiredWheelAngle[E_RearLeft]);
    frc::SmartDashboard::PutNumber("Angle Desire Rear Right", V_DesiredWheelAngle[E_RearRight]);


    
    frc::Wait(0.1);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif