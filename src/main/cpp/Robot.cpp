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

double V_WS[E_RobotCornerSz];
double V_WA[E_RobotCornerSz];

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
double V_FWD;
double V_STR;
double V_RCW;

frc::LiveWindow *lw = frc::LiveWindow::GetInstance();
std::shared_ptr<NetworkTable> vision;
nt::NetworkTableInstance inst;
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
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

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

    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the Follow() method on the SPARK MAX you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     *
     * This is shown in the example below, where one motor on each side of our drive train is
     * configured to follow a lead motor.
     */
//    m_leftFollowMotor.Follow(m_leftLeadMotor);
//    m_rightFollowMotor.Follow(m_rightLeadMotor);
        
    inst = nt::NetworkTableInstance::Create();
    inst.StartClient("10.55.61.24");
    inst.StartDSClient();
  
    vision = inst.GetTable("chameleon-vision/Scotty");
    
    driverMode = vision->GetEntry("driver_mode");
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit(){
  V_RobotInit = true;
  V_WheelSpeedDelay = false;
  V_ModeTransition = false;
  V_Mode = 0;

  for (int index = E_FrontLeft;
         index < E_RobotCornerSz;
         index = T_RobotCorner(int(index) + 1))
      {
        V_WS[index] = 0;
        V_WA[index] = 0;
        V_DesiredWheelAngle[index] = 0;
        V_WheelRelativeAngleRawOffset[index] = 0;
        V_WheelAngle[index] = 0;
      }
      V_STR = 0;
      V_FWD = 0;
      gyro_yawangledegrees = 0;
      gyro_yawanglerad = 0;

  GyroTeleInit();
}

void Robot::TeleopPeriodic() {
  
  ColorSensor(false);
  Gyro();
  V_FWD = c_joyStick.GetRawAxis(1) * -1;
  V_STR = c_joyStick.GetRawAxis(0);
  V_RCW = c_joyStick.GetRawAxis(4);


  V_FWD = DesiredSpeed(V_FWD);
  V_STR = DesiredSpeed(V_STR);
  V_RCW = DesiredSpeed(V_RCW);

  double L_temp = V_FWD * cos(gyro_yawanglerad) + V_STR * sin(gyro_yawanglerad);
  V_STR = -V_FWD * sin(gyro_yawanglerad) + V_STR * cos(gyro_yawanglerad);
  V_FWD = L_temp;

      //Ws1: fr, Ws2: fl, ws3: rl, ws4: rr
  double V_A = V_STR - V_RCW * (C_L/C_R);
  double V_B = V_STR + V_RCW * (C_L/C_R);
  double V_C = V_FWD - V_RCW * (C_W/C_R);
  double V_D = V_FWD + V_RCW * (C_W/C_R);

  V_WS[E_FrontRight] = pow((V_B * V_B + V_C * V_C), 0.5);
  V_WS[E_FrontLeft] = pow((V_B * V_B + V_D * V_D), 0.5);
  V_WS[E_RearLeft] = pow((V_A * V_A + V_D * V_D), 0.5);
  V_WS[E_RearRight] = pow((V_A * V_A + V_C * V_C), 0.5);

  V_WA[E_FrontRight] = atan2(V_B, V_C) *180/C_PI;
  V_WA[E_FrontLeft] = atan2(V_B, V_D) *180/C_PI;
  V_WA[E_RearLeft] = atan2(V_A, V_D) *180/C_PI;
  V_WA[E_RearRight] = atan2(V_A, V_C) *180/C_PI;

  double V_Max = V_WS[E_FrontRight];

  if (V_WS[E_FrontLeft] > V_Max) {
      V_Max = V_WS[E_FrontLeft];
  }
 if (V_WS[E_RearLeft] > V_Max) {
      V_Max = V_WS[E_RearLeft];
  }
   if (V_WS[E_RearRight] > V_Max) {
      V_Max = V_WS[E_RearRight];
  }
  if (V_Max > 1) {
      V_WS[E_FrontRight] /= V_Max;
      V_WS[E_FrontLeft] /= V_Max;
      V_WS[E_RearLeft] /= V_Max;
      V_WS[E_RearRight] /= V_Max;
  }

  V_WS[E_FrontRight] *= 20;
  V_WS[E_FrontLeft] *= 20;
  V_WS[E_RearLeft] *= 20;
  V_WS[E_RearRight] *= 20;
  
  frc::SmartDashboard::PutNumber("Gyro Angle Deg", gyro_yawangledegrees);
  frc::SmartDashboard::PutNumber("Gyro Angle Rad", gyro_yawanglerad);

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

  //8.31 : 1 
  //11.9 m/s

  T_RobotCorner index;
  bool L_WheelSpeedDelay = false;


    for (index = E_FrontLeft; index < E_RobotCornerSz; index = T_RobotCorner(int(index) + 1)){

    }

    Read_Encoders(V_RobotInit, 
    a_encoderFrontLeftSteer.GetVoltage(), a_encoderFrontRightSteer.GetVoltage(), a_encoderRearLeftSteer.GetVoltage(), a_encoderRearRightSteer.GetVoltage(), 
    m_encoderFrontLeftSteer,m_encoderFrontRightSteer, m_encoderRearLeftSteer, m_encoderRearRightSteer, 
    m_encoderFrontLeftDrive, m_encoderFrontRightDrive,m_encoderRearLeftDrive, m_encoderRearRightDrive);

    if ((c_joyStick.GetRawAxis(2) > 0.1) || (c_joyStick.GetRawAxis(3) > 0.1)) // Rotate clockwise w/ 2, counter clockwise w/ 3
      {
      V_DesiredWheelAngle[E_FrontLeft] = 45;
      V_DesiredWheelAngle[E_FrontRight] = 135;
      V_DesiredWheelAngle[E_RearLeft] = -45;
      V_DesiredWheelAngle[E_RearRight] = -135;

      if (V_Mode != 1)
        {
        V_WheelSpeedDelay = true;
        }

      if (V_WheelSpeedDelay == true)
        {
        L_WheelSpeedDelay = false;
        V_WheelSpeedDelay = false;

        for (index = E_FrontLeft;
             index < E_RobotCornerSz;
             index = T_RobotCorner(int(index) + 1))
          {
          L_WheelSpeedDelay =  CriteriaMet(V_DesiredWheelAngle[index],
                                           V_WheelAngle[index],
                                           5);
          if (L_WheelSpeedDelay == true)
            {
            V_WheelSpeedDelay = true;
            }
          }
        }
      else
        {
        for (index = E_FrontLeft;
             index < E_RobotCornerSz;
             index = T_RobotCorner(int(index) + 1))
          {
            // if (index == E_RearRight)
            // {V_WheelSpeedCmnd[index] =  (c_joyStick.GetRawAxis(2) - c_joyStick.GetRawAxis(3)) * 0.3;}
            // else
            // {
              // V_WheelSpeedCmnd[index] =  (c_joyStick.GetRawAxis(2) - c_joyStick.GetRawAxis(3)) * -0.3;
            // }
            V_WheelRpmCmnd[index] =  (c_joyStick.GetRawAxis(2) - c_joyStick.GetRawAxis(3)) * -20;
          
          }
        }

      V_Mode = 1;
      }
    else if ((c_joyStick.GetRawButton(5) == true) || (c_joyStick.GetRawButton(6) == true)) // Straif left 5, right 6
      {
      for (index = E_FrontLeft;
           index < E_RobotCornerSz;
           index = T_RobotCorner(int(index) + 1))
        {
        if (c_joyStick.GetRawButton(5) == true)
          {
          V_DesiredWheelAngle[index] = -90;
          }
        else
          {
          V_DesiredWheelAngle[index] = 90;
          }
        }

      if (V_Mode != 2)
        {
        V_WheelSpeedDelay = true;
        }

      if (V_WheelSpeedDelay == true)
        {
        L_WheelSpeedDelay = false;
        V_WheelSpeedDelay = false;

        for (index = E_FrontLeft;
             index < E_RobotCornerSz;
             index = T_RobotCorner(int(index) + 1))
          {
          L_WheelSpeedDelay =  CriteriaMet(V_DesiredWheelAngle[index],
                                           V_WheelAngle[index],
                                           5);
          if (L_WheelSpeedDelay == true)
            {
            V_WheelSpeedDelay = true;
            }
          }
        }

      for (index = E_FrontLeft;
           index < E_RobotCornerSz;
           index = T_RobotCorner(int(index) + 1))
        {
        if (V_WheelSpeedDelay == false)
          {
          V_WheelRpmCmnd[index] = 20;
          }
        else
          {
          V_WheelRpmCmnd[index] = 0.0;
          }
        }
      V_Mode = 2;
      }
    else
      {
      
      double _JoyStickX = c_joyStick.GetRawAxis(0);
      double _JoyStickY = c_joyStick.GetRawAxis(1);
      double _JoyAngle;
      double _JoyStickZ = sqrt((_JoyStickX * _JoyStickX) + (_JoyStickY * _JoyStickY));
      if(_JoyStickY > 0.01) {
        _JoyAngle = RadtoDeg * atan(_JoyStickX/-_JoyStickY);  
      } else if(_JoyStickY < -0.01) {
        _JoyAngle = RadtoDeg * atan(-_JoyStickX/_JoyStickY);  
      } else {
        _JoyAngle = _JoyStickX * 90;
      }
       
       if (_JoyStickY < 0) {
         _JoyStickZ = -_JoyStickZ;
       }
       else if (_JoyStickY > 0) {
         _JoyStickZ = _JoyStickZ;
       }
      
      
      

      for (index = E_FrontLeft;
           index < E_RobotCornerSz;
           index = T_RobotCorner(int(index) + 1))
        {
        
        V_DesiredWheelAngle[index] = _JoyAngle;
        V_WheelRpmCmnd[index] = _JoyStickZ * -20;
        }
      V_WheelSpeedDelay = false;
      V_Mode = 3;
      }

    
    if (V_RobotInit == true)
      {
      V_RobotInit = false;
      for (index = E_FrontLeft;
           index < E_RobotCornerSz;
           index = T_RobotCorner(int(index) + 1))
        {
        V_WS[index] = 0;
        V_WA[index] = 0;
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
                                              0.005, // P Gx
                                              0.00008, // I Gx
                                              0.0, // D Gx
                                              0.15, // P UL
                                             -0.15, // P LL
                                              0.015, // I UL
                                             -0.015, // I LL
                                              0.0, // D UL
                                             -0.0, // D LL
                                              0.7, // Max upper
                                             -0.7); // Max lower

      V_WheelSpeedCmnd[index] = Control_PID(V_WS[index],
                                      V_WheelVelocity[index], 
                                      &V_WheelSpeedError[index], 
                                      &V_WheelSpeedIntergral[index], 
                                      0.007, // P Gx
                                      0.0005, // I Gx
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

    frc::SmartDashboard::PutNumber("WS_FR", V_WS[E_FrontRight]);
    frc::SmartDashboard::PutNumber("WS_FL", V_WS[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("WS_RL", V_WS[E_RearLeft]);
    frc::SmartDashboard::PutNumber("WS_RR", V_WS[E_RearRight]);

    frc::SmartDashboard::PutNumber("WA_FR", V_WA[E_FrontRight]);
    frc::SmartDashboard::PutNumber("WA_FL", V_WA[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("WA_RL", V_WA[E_RearLeft]);
    frc::SmartDashboard::PutNumber("WA_RR", V_WA[E_RearRight]);

    m_frontLeftDriveMotor.Set(V_WheelSpeedCmnd[E_FrontLeft]);
    m_frontRightDriveMotor.Set(V_WheelSpeedCmnd[E_FrontRight]);
    m_rearLeftDriveMotor.Set(V_WheelSpeedCmnd[E_RearLeft]);
    m_rearRightDriveMotor.Set(V_WheelSpeedCmnd[E_RearRight]);

    m_frontLeftSteerMotor.Set(V_WheelAngleCmnd[E_FrontLeft] * (-1));
    m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight] * (-1));
    m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft] * (-1));
    m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight] * (-1));

    // m_frontLeftSteerMotor.Set (0);
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
    frc::SmartDashboard::PutBoolean("driver_mode", driverMode.GetBoolean(false));
   
    frc::SmartDashboard::PutNumber("Angle Desire Front Left", V_DesiredWheelAngle[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("Angle Desire Front Right", V_DesiredWheelAngle[E_FrontRight]);
    frc::SmartDashboard::PutNumber("Angle Desire Rear Left", V_DesiredWheelAngle[E_RearLeft]);
    frc::SmartDashboard::PutNumber("Angle Desire Rear Right", V_DesiredWheelAngle[E_RearRight]);

    // /**
    //  * Open Smart Dashboard or Shuffleboard to see the color detected by the 
    //  * sensor.
    //  */
    //lw->Add
  
    //frc::SmartDashboard::PutNumberArray("Anglealjshdfkjshdfsdhfklsjdfh Desired Array", V_DesiredWheelAngle);

    frc::SmartDashboard::PutBoolean("Wheel Delay",  V_WheelSpeedDelay);
    frc::SmartDashboard::PutBoolean("RobotInit",  V_RobotInit);
    frc::SmartDashboard::PutNumber("Desired Wheel Angle",  V_DesiredWheelAngle[E_FrontLeft]);

    
    frc::Wait(0.1);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif