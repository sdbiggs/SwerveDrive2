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

#include "Encoder.hpp"
#include "Enums.hpp"
#include "AHRS.h"

double V_WheelRPM[E_RobotCornerSz];
double V_WheelAngle[E_RobotCornerSz];
double V_WheelRelativeAngleRawOffset[E_RobotCornerSz];
double V_WheelAngleCmnd[E_RobotCornerSz];
double V_WheelSpeedCmnd[E_RobotCornerSz];
double V_WheelAngleError[E_RobotCornerSz];
double V_WheelAngleIntegral[E_RobotCornerSz];
double V_DesiredWheelAngle[E_RobotCornerSz];
double V_DesiredWheelSpeed[E_RobotCornerSz];
bool   V_RobotInit;
bool   V_ModeTransition;
int    V_Mode;
bool   V_WheelSpeedDelay;
int gryoloopcnt = 0;
float gyroangleprev;

AHRS *NavX;

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


/******************************************************************************
 * Function:     Control_PID
 *
 * Description:  This function provides PID control.  This will also limit the
 *               the three controllers (P I D) by the calibrated thresholds.
 ******************************************************************************/
double Control_PID(double  L_DesiredSpeed,
                   double  L_CurrentSpeed,
                   double *L_ErrorPrev,
                   double *L_IntegralPrev,
                   double  L_ProportionalGx,
                   double  L_IntegralGx,
                   double  L_DerivativeGx,
                   double  L_ProportionalUpperLimit,
                   double  L_ProportionalLowerLimit,
                   double  L_IntegralUpperLimit,
                   double  L_IntegralLowerLimit,
                   double  L_DerivativeUpperLimit,
                   double  L_DerivativeLowerLimit,
                   double  L_OutputUpperLimit,
                   double  L_OutputLowerLimit)
  {
  double L_Error        = 0.0;
  double L_Proportional = 0.0;
  double L_Integral     = 0.0;
  double L_Derivative   = 0.0;
  double L_OutputCmnd   = 0.0;

  L_Error = L_DesiredSpeed - L_CurrentSpeed;

  L_Proportional = L_Error * L_ProportionalGx;

  L_Integral = *L_IntegralPrev + (L_Error * L_IntegralGx);

  L_Derivative = L_DerivativeGx * (*L_ErrorPrev / 0.01);

  *L_ErrorPrev = L_Error;

  if (L_Proportional > L_ProportionalUpperLimit)
    {
    L_Proportional = L_ProportionalUpperLimit;
    }
  else if (L_Proportional < L_ProportionalLowerLimit)
    {
    L_Proportional = L_ProportionalLowerLimit;
    }

  if (L_Integral > L_IntegralUpperLimit)
    {
    L_Integral = L_IntegralUpperLimit;
    }
  else if (L_Integral < L_IntegralLowerLimit)
    {
    L_Integral = L_IntegralLowerLimit;
    }

  if (L_Derivative > L_DerivativeUpperLimit)
    {
    L_Derivative = L_DerivativeUpperLimit;
    }
  else if (L_Derivative < L_DerivativeLowerLimit)
    {
    L_Derivative = L_DerivativeLowerLimit;
    }

  /* Ok, lets record the integral to use next loop: */
  *L_IntegralPrev = L_Integral;

  /* Lets add all three controllers. */
  L_OutputCmnd = L_Proportional + L_Integral + L_Derivative;

  /* This is kind of redundant, but lets limit the output to the min and max
   * allowed for the controller: */
  if (L_OutputCmnd > L_OutputUpperLimit)
    {
    L_OutputCmnd = L_OutputUpperLimit;
    }
  else if (L_OutputCmnd < L_OutputLowerLimit)
    {
    L_OutputCmnd = L_OutputLowerLimit;
    }

  return L_OutputCmnd;
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


    try{
      NavX = new AHRS(SPI::Port::kMXP);
    }
    catch(const std::exception e){
      std::string err_string = "Error instantiating navX-MXP:  ";
      err_string += e.what();
      DriverStation::ReportError(err_string.c_str());
    }
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
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
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

  NavX->ZeroYaw();
}

void Robot::TeleopPeriodic() {
  
  float currentyaw = NavX->GetYaw();
  
  //Check to see if gyro angle flips over 180 or -180
  if(175 < abs(gyroangleprev))
  {
    if(currentyaw < 0)
    {
      gryoloopcnt -= 1;
    } else if (currentyaw > 0)
    {
      gryoloopcnt += 1;
    }
  }

  float finalangle = (gryoloopcnt * 360) + currentyaw;

  SmartDashboard::PutNumber("NavX Raw Yaw", NavX->GetYaw());

  gyroangleprev = currentyaw;

  T_RobotCorner index;
  bool L_WheelSpeedDelay = false;

    if ((c_joyStick.GetRawAxis(2) > 0.1) || (c_joyStick.GetRawAxis(3) > 0.1)) // Rotate clockwise w/ 2, counter clockwise w/ 3
      {
      V_DesiredWheelAngle[E_FrontLeft] = 45;
      V_DesiredWheelAngle[E_FrontRight] = 135;
      V_DesiredWheelAngle[E_RearLeft] = -45;
      V_DesiredWheelAngle[E_RearRight] = 225;

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
          V_WheelSpeedCmnd[index] =  (c_joyStick.GetRawAxis(2) - c_joyStick.GetRawAxis(3)) * -0.3;
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
          V_WheelSpeedCmnd[index] = 0.2;
          }
        else
          {
          V_WheelSpeedCmnd[index] = 0.0;
          }
        }
      V_Mode = 2;
      }
    else
      {
      for (index = E_FrontLeft;
           index < E_RobotCornerSz;
           index = T_RobotCorner(int(index) + 1))
        {
        V_DesiredWheelAngle[index] = c_joyStick.GetRawAxis(0) * 90;
        V_WheelSpeedCmnd[index] = c_joyStick.GetY() * -0.5;
        }
      V_WheelSpeedDelay = false;
      V_Mode = 3;
      }

    if (V_RobotInit == true)
      {
      V_WheelAngle[E_FrontLeft]  = a_encoderFrontLeftSteer.GetVoltage() * 72 - 309;
      V_WheelAngle[E_FrontRight] = a_encoderFrontRightSteer.GetVoltage() * 72 - 106.7;
      V_WheelAngle[E_RearLeft]   = a_encoderRearLeftSteer.GetVoltage() * 72 - 7;
      V_WheelAngle[E_RearRight]  = a_encoderRearRightSteer.GetVoltage() * 72 - 178;

      V_DesiredWheelAngle[E_FrontLeft] = 0;
      V_DesiredWheelAngle[E_FrontRight] = 0;
      V_DesiredWheelAngle[E_RearLeft] = 0;
      V_DesiredWheelAngle[E_RearRight] = 0;
      }
    else
      {
      V_WheelAngle[E_FrontLeft] = (m_encoderFrontLeftSteer.GetPosition() - V_WheelRelativeAngleRawOffset[E_FrontLeft]) * -20;
      V_WheelAngle[E_FrontRight] = (m_encoderFrontRightSteer.GetPosition() - V_WheelRelativeAngleRawOffset[E_FrontRight]) * -20;
      V_WheelAngle[E_RearLeft] = (m_encoderRearLeftSteer.GetPosition() - V_WheelRelativeAngleRawOffset[E_RearLeft]) * -20;
      V_WheelAngle[E_RearRight] = (m_encoderRearRightSteer.GetPosition() - V_WheelRelativeAngleRawOffset[E_RearRight]) * -20;
      }

    for (index = E_FrontLeft;
         index < E_RobotCornerSz;
         index = T_RobotCorner(int(index) + 1))
      {
      V_WheelAngleCmnd[index] =  Control_PID( V_DesiredWheelAngle[index],
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
      }

    if (V_RobotInit == true)
      {
      V_RobotInit = false;
      for (index = E_FrontLeft;
           index < E_RobotCornerSz;
           index = T_RobotCorner(int(index) + 1))
        {
        if (fabs(V_WheelAngle[index]) > 1.2)
          {
          V_RobotInit = true;
          }
        }

      if (V_RobotInit == false)
        {
        V_WheelRelativeAngleRawOffset[E_FrontLeft] = m_encoderFrontLeftSteer.GetPosition();
        V_WheelRelativeAngleRawOffset[E_FrontRight] = m_encoderFrontRightSteer.GetPosition();
        V_WheelRelativeAngleRawOffset[E_RearLeft] = m_encoderRearLeftSteer.GetPosition();
        V_WheelRelativeAngleRawOffset[E_RearRight] = m_encoderRearRightSteer.GetPosition();
        }
      }

    m_frontLeftDriveMotor.Set(V_WheelSpeedCmnd[E_FrontLeft]);
    m_frontRightDriveMotor.Set(V_WheelSpeedCmnd[E_FrontRight]);
    m_rearLeftDriveMotor.Set(V_WheelSpeedCmnd[E_RearLeft]);
    m_rearRightDriveMotor.Set(V_WheelSpeedCmnd[E_RearRight]);

    m_frontLeftSteerMotor.Set(V_WheelAngleCmnd[E_FrontLeft] * (-1));
    m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight] * (-1));
    m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft] * (-1));
    m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight] * (-1));

//    m_frontLeftSteerMotor.Set(V_WheelAngleCmnd[E_FrontLeft] * (0));
//    m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight] * (0));
//    m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft] * (0));
//    m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight] * (0));


    frc::SmartDashboard::PutBoolean("Wheel Delay",  V_WheelSpeedDelay);
    frc::SmartDashboard::PutBoolean("RobotInit",  V_RobotInit);
    frc::SmartDashboard::PutNumber("Desired Wheel Angle",  V_DesiredWheelAngle[E_FrontLeft]);

    frc::SmartDashboard::PutNumber("Encoder Front Left Converted",  V_WheelAngle[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("Encoder Front Right Converted", V_WheelAngle[E_FrontRight]);
    frc::SmartDashboard::PutNumber("Encoder Rear Left Converted",   V_WheelAngle[E_RearLeft]);
    frc::SmartDashboard::PutNumber("Encoder Rear Right Converted",  V_WheelAngle[E_RearRight]);

    frc::SmartDashboard::PutNumber("Encoder Front Left Steer Position", m_encoderFrontLeftSteer.GetPosition());
    frc::SmartDashboard::PutNumber("Encoder Front Left Steer Velocity", m_encoderFrontLeftSteer.GetVelocity());

    frc::SmartDashboard::PutNumber("Encoder Front Left Drive Position", m_encoderFrontLeftDrive.GetPosition());
    frc::SmartDashboard::PutNumber("Encoder Front Left Drive Velocity", m_encoderFrontLeftDrive.GetVelocity());

    frc::SmartDashboard::PutNumber("Encoder Front Right Steer Position", m_encoderFrontRightSteer.GetPosition());
    frc::SmartDashboard::PutNumber("Encoder Front Right Steer Velocity", m_encoderFrontRightSteer.GetVelocity());

    frc::SmartDashboard::PutNumber("Encoder Front Right Drive Position", m_encoderFrontRightDrive.GetPosition());
    frc::SmartDashboard::PutNumber("Encoder Front Right Drive Velocity", m_encoderFrontRightDrive.GetVelocity());

    frc::SmartDashboard::PutNumber("Encoder Rear Left Steer Position", m_encoderRearLeftSteer.GetPosition());
    frc::SmartDashboard::PutNumber("Encoder Rear Left Steer Velocity", m_encoderRearLeftSteer.GetVelocity());

    frc::SmartDashboard::PutNumber("Encoder Rear Left Drive Position", m_encoderRearLeftDrive.GetPosition());
    frc::SmartDashboard::PutNumber("Encoder Rear Left Drive Velocity", m_encoderRearLeftDrive.GetVelocity());

    frc::SmartDashboard::PutNumber("Encoder Rear Right Steer Position", m_encoderRearRightSteer.GetPosition());
    frc::SmartDashboard::PutNumber("Encoder Rear Right Steer Velocity", m_encoderRearRightSteer.GetVelocity());

    frc::SmartDashboard::PutNumber("Encoder Rear Right Drive Position", m_encoderRearRightDrive.GetPosition());
    frc::SmartDashboard::PutNumber("Encoder Rear Right Drive Velocity", m_encoderRearRightDrive.GetVelocity());

    frc::SmartDashboard::PutNumber("Encoder Front Left", a_encoderFrontLeftSteer.GetVoltage());
    frc::SmartDashboard::PutNumber("Encoder Front Right", a_encoderFrontRightSteer.GetVoltage());
    frc::SmartDashboard::PutNumber("Encoder Rear Left", a_encoderRearLeftSteer.GetVoltage());
    frc::SmartDashboard::PutNumber("Encoder Rear Right", a_encoderRearRightSteer.GetVoltage());
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
