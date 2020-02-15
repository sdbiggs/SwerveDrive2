/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc/AnalogInput.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include "Const.hpp"
#include "ctre/Phoenix.h"
 
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  frc::AnalogInput a_encoderFrontLeftSteer{2};
  frc::AnalogInput a_encoderFrontRightSteer{1};
  frc::AnalogInput a_encoderRearLeftSteer{3};
  frc::AnalogInput a_encoderRearRightSteer{0};

  rev::CANSparkMax m_frontLeftSteerMotor {frontLeftSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontLeftDriveMotor {frontLeftDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontRightSteerMotor{frontRightSteerDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontRightDriveMotor{frontRightDriveDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearLeftSteerMotor  {rearLeftSteerDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearLeftDriveMotor  {rearLeftDriveDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearRightSteerMotor {rearRightSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearRightDriveMotor {rearRightDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_topShooterMotor     {topShooterID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_bottomShooterMotor  {bottomShooterID,  rev::CANSparkMax::MotorType::kBrushless};

  VictorSPX m_intake {12};

  rev::CANEncoder m_encoderFrontLeftSteer  = m_frontLeftSteerMotor.GetEncoder();
  rev::CANEncoder m_encoderFrontLeftDrive  = m_frontLeftDriveMotor.GetEncoder();
  rev::CANEncoder m_encoderFrontRightSteer = m_frontRightSteerMotor.GetEncoder();
  rev::CANEncoder m_encoderFrontRightDrive = m_frontRightDriveMotor.GetEncoder();
  rev::CANEncoder m_encoderRearLeftSteer   = m_rearLeftSteerMotor.GetEncoder();
  rev::CANEncoder m_encoderRearLeftDrive   = m_rearLeftDriveMotor.GetEncoder();
  rev::CANEncoder m_encoderRearRightSteer  = m_rearRightSteerMotor.GetEncoder();
  rev::CANEncoder m_encoderRearRightDrive  = m_rearRightDriveMotor.GetEncoder();
  rev::CANEncoder m_encoderTopShooter      = m_topShooterMotor.GetEncoder();
  rev::CANEncoder m_encoderBottomShooter   = m_bottomShooterMotor.GetEncoder();

  frc::Joystick c_joyStick{0};
  frc::Joystick a_joyStick{1};

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  
};
