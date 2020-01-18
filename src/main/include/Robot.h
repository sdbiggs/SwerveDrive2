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
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

 
class Robot : public frc::TimedRobot {
  static const int frontLeftSteerDeviceID = 1, frontLeftDriveDeviceID = 2, frontRightSteerDeviceID = 4, frontRightDriveDeviceID = 3;
  static const int rearLeftSteerDeviceID  = 5, rearLeftDriveDeviceID  = 6, rearRightSteerDeviceID  = 7, rearRightDriveDeviceID  = 8;

  rev::CANSparkMax m_frontLeftSteerMotor {frontLeftSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontLeftDriveMotor {frontLeftDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontRightSteerMotor{frontRightSteerDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontRightDriveMotor{frontRightDriveDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearLeftSteerMotor  {rearLeftSteerDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearLeftDriveMotor  {rearLeftDriveDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearRightSteerMotor {rearRightSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearRightDriveMotor {rearRightDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};

  rev::CANEncoder m_encoderFrontLeftSteer  = m_frontLeftSteerMotor.GetEncoder();
  rev::CANEncoder m_encoderFrontLeftDrive  = m_frontLeftDriveMotor.GetEncoder();
  rev::CANEncoder m_encoderFrontRightSteer = m_frontRightSteerMotor.GetEncoder();
  rev::CANEncoder m_encoderFrontRightDrive = m_frontRightDriveMotor.GetEncoder();
  rev::CANEncoder m_encoderRearLeftSteer   = m_rearLeftSteerMotor.GetEncoder();
  rev::CANEncoder m_encoderRearLeftDrive   = m_rearLeftDriveMotor.GetEncoder();
  rev::CANEncoder m_encoderRearRightSteer  = m_rearRightSteerMotor.GetEncoder();
  rev::CANEncoder m_encoderRearRightDrive  = m_rearRightDriveMotor.GetEncoder();

  frc::AnalogInput a_encoderFrontLeftSteer{0};
  frc::AnalogInput a_encoderFrontRightSteer{1};
  frc::AnalogInput a_encoderRearLeftSteer{2};
  frc::AnalogInput a_encoderRearRightSteer{3};

  frc::Joystick c_joyStick{0};

   static constexpr auto i2cPort = frc::I2C::Port::kOnboard;


 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  rev::ColorMatch m_colorMatcher;
  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
};
