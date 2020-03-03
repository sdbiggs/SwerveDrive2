#ifndef ROBOT
#define ROBOT

#pragma once

#include <string>

#include <frc/AnalogInput.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/TimedRobot.h>
#include <frc/PowerDistributionPanel.h>


#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
 
#include "Const.hpp"

#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
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
  rev::CANSparkMax m_liftMotor           {liftID,                  rev::CANSparkMax::MotorType::kBrushless};

  VictorSPX m_conveyDaBalls {12};
  VictorSPX m_fortuneWheel {13};
  VictorSPX m_intake {15};
  VictorSPX m_elevateDaBalls {16};

  frc::Compressor compressor {14};

  frc::DoubleSolenoid lift {14, 0, 1};
  frc::DoubleSolenoid intake {14, 2, 3};
  
  rev::CANPIDController m_topShooterpid = m_topShooterMotor.GetPIDController();
  rev::CANPIDController m_bottomShooterpid = m_bottomShooterMotor.GetPIDController();
  rev::CANPIDController m_liftpid          = m_liftMotor.GetPIDController();



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
  rev::CANEncoder m_encoderLift            = m_liftMotor.GetEncoder();

  frc::Joystick c_joyStick{0};
  frc::Joystick c_joyStick2{1};

  frc::PowerDistributionPanel PDP {0};

  double Upper_P_Gx = 0, Upper_I_Gx = 0, Upper_D_Gx = 0, Upper_I_Zone = 0, Upper_FF = 0, Upper_Max = 1, Upper_Min = -1;
  double Lower_P_Gx = 0, Lower_I_Gx = 0, Lower_D_Gx = 0, Lower_I_Zone = 0, Lower_FF = 0, Lower_Max = 1, Lower_Min = -1;
  double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  
};

#endif