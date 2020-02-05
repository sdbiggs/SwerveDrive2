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
double V_WA_Prev[E_RobotCornerSz];
double V_WA_Loopcount[E_RobotCornerSz];
double V_WA_Final[E_RobotCornerSz];

double V_WheelRPM[E_RobotCornerSz];

double V_WheelAngleCmnd[E_RobotCornerSz];
double V_WheelSpeedCmnd[E_RobotCornerSz];
double V_WheelSpeedIntergral[E_RobotCornerSz];
double V_WheelRpmCmnd[E_RobotCornerSz];
double V_WheelAngleError[E_RobotCornerSz];
double V_WheelAngleIntegral[E_RobotCornerSz];
bool   V_RobotInit;
bool   V_ModeTransition;
int    V_Mode;
bool   V_WheelSpeedDelay;
double V_WheelSpeedError[E_RobotCornerSz];
double V_JoystickAxisForward;
double V_JoystickAxisStrafe;
double V_JoystickAxisRotate;

std::shared_ptr<NetworkTable> vision;
nt::NetworkTableInstance inst;
nt::NetworkTableEntry driverMode;

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
      V_JoystickAxisStrafe = 0;
      V_JoystickAxisForward = 0;
      gyro_yawangledegrees = 0;
      gyro_yawanglerad = 0;

  GyroTeleInit();
}

void Robot::TeleopPeriodic() {
  
  ColorSensor(false);
  Gyro(); 
  bool L_WheelSpeedDelay = false;

  Read_Encoders(V_RobotInit, 
  a_encoderFrontLeftSteer.GetVoltage(), a_encoderFrontRightSteer.GetVoltage(), a_encoderRearLeftSteer.GetVoltage(), a_encoderRearRightSteer.GetVoltage(), 
  m_encoderFrontLeftSteer,m_encoderFrontRightSteer, m_encoderRearLeftSteer, m_encoderRearRightSteer, 
  m_encoderFrontLeftDrive, m_encoderFrontRightDrive,m_encoderRearLeftDrive, m_encoderRearRightDrive);

  //Swerve Math
  #pragma region 
  V_JoystickAxisForward = c_joyStick.GetRawAxis(1) * -1;
  V_JoystickAxisStrafe = c_joyStick.GetRawAxis(0);
  V_JoystickAxisRotate = c_joyStick.GetRawAxis(4);

  V_JoystickAxisForward = DesiredSpeed(V_JoystickAxisForward);
  V_JoystickAxisStrafe = DesiredSpeed(V_JoystickAxisStrafe);
  V_JoystickAxisRotate = DesiredSpeed(V_JoystickAxisRotate);

  double L_temp = V_JoystickAxisForward * cos(gyro_yawanglerad) + V_JoystickAxisStrafe * sin(gyro_yawanglerad);
  V_JoystickAxisStrafe = -V_JoystickAxisForward * sin(gyro_yawanglerad) + V_JoystickAxisStrafe * cos(gyro_yawanglerad);
  V_JoystickAxisForward = L_temp;

      //Ws1: fr, Ws2: fl, ws3: rl, ws4: rr
  double V_A = V_JoystickAxisStrafe - V_JoystickAxisRotate * (C_Lenght/C_R);
  double V_B = V_JoystickAxisStrafe + V_JoystickAxisRotate * (C_Lenght/C_R);
  double V_C = V_JoystickAxisForward - V_JoystickAxisRotate * (C_Width/C_R);
  double V_D = V_JoystickAxisForward + V_JoystickAxisRotate * (C_Width/C_R);

  V_WS[E_FrontRight] = pow((V_B * V_B + V_C * V_C), 0.5);
  V_WS[E_FrontLeft] = pow((V_B * V_B + V_D * V_D), 0.5);
  V_WS[E_RearLeft] = pow((V_A * V_A + V_D * V_D), 0.5);
  V_WS[E_RearRight] = pow((V_A * V_A + V_C * V_C), 0.5);

  T_RobotCorner index = E_FrontLeft;
  for (index = E_FrontLeft; index < E_RobotCornerSz; index = T_RobotCorner(int(index) + 1)){
      V_WA_Prev[index] = V_WA[index];
    }
    
  V_WA[E_FrontRight] = atan2(V_B, V_C) *180/C_PI;
  V_WA[E_FrontLeft] = atan2(V_B, V_D) *180/C_PI;
  V_WA[E_RearLeft] = atan2(V_A, V_D) *180/C_PI;
  V_WA[E_RearRight] = atan2(V_A, V_C) *180/C_PI;
  #pragma endregion

  //Scotts loop count code 
  #pragma region 

// for (index = E_FrontLeft;
//      index < E_RobotCornerSz;
//      index = T_RobotCorner(int(index) + 1))
//   {
//     if(abs(V_WA_Prev[index]) >= 150)
//       {
//         if(V_WA_Prev[index] < 0 && V_WA[index] > 0)
//           {
//            V_WA_Loopcount[index] += 1;
//           }     
//         else if (V_WA_Prev[index] > 0 && V_WA[index] < 0)
//           {
//            V_WA_Loopcount[index] -= 1;
//           }
//      }
//     V_WA_Final[index] = (V_WA_Loopcount[index] * 360) + V_WA[index];
//     //V_WA_Final[index] = V_WA[index];
//   }
  #pragma endregion

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
  
  //Wes's swerve upgrade?
  #pragma region 
  //PS positive angle in 0 to 180
  //NS Negative angle in 0 to -180
  double NS = 0;
  double PS = 0;
  double AnglePS = 0; 
  double AngleNS = 0;

  for (index = E_FrontLeft; index < E_RobotCornerSz; index = T_RobotCorner(int(index) + 1)){
    //check to set if angle is negative
    if(V_WA[index] < 0)
    {
      NS = V_WA[index];
      PS = 180 - abs(V_WA[index]);
    } 
    if(V_WA[index] > 0)
    {
      NS = -(180 - abs(V_WA[index]));
      PS = V_WA[index];
    }

    AngleNS = abs(V_WheelAngle[index] - NS);
    AnglePS = abs(V_WheelAngle[index] - PS);

    if(AngleNS < AnglePS)
    {
      V_WA[index] = NS;
      V_WS[index] *= -1;
    }
    if(AngleNS > AnglePS)
    {
      V_WA[index] = PS;
    }

  }
  #pragma endregion
   

    //old code
  #pragma region

    // if ((c_joyStick.GetRawAxis(2) > 0.1) || (c_joyStick.GetRawAxis(3) > 0.1)) // Rotate clockwise w/ 2, counter clockwise w/ 3
    //   {
    //   V_DesiredWheelAngle[E_FrontLeft] = 45;
    //   V_DesiredWheelAngle[E_FrontRight] = 135;
    //   V_DesiredWheelAngle[E_RearLeft] = -45;
    //   V_DesiredWheelAngle[E_RearRight] = -135;

    //   if (V_Mode != 1)
    //     {
    //     V_WheelSpeedDelay = true;
    //     }

    //   if (V_WheelSpeedDelay == true)
    //     {
    //     L_WheelSpeedDelay = false;
    //     V_WheelSpeedDelay = false;

    //     for (index = E_FrontLeft;
    //          index < E_RobotCornerSz;
    //          index = T_RobotCorner(int(index) + 1))
    //       {
    //       L_WheelSpeedDelay =  CriteriaMet(V_DesiredWheelAngle[index],
    //                                        V_WheelAngle[index],
    //                                        5);
    //       if (L_WheelSpeedDelay == true)
    //         {
    //         V_WheelSpeedDelay = true;
    //         }
    //       }
    //     }
    //   else
    //     {
    //     for (index = E_FrontLeft;
    //          index < E_RobotCornerSz;
    //          index = T_RobotCorner(int(index) + 1))
    //       {
    //         // if (index == E_RearRight)
    //         // {V_WheelSpeedCmnd[index] =  (c_joyStick.GetRawAxis(2) - c_joyStick.GetRawAxis(3)) * 0.3;}
    //         // else
    //         // {
    //           // V_WheelSpeedCmnd[index] =  (c_joyStick.GetRawAxis(2) - c_joyStick.GetRawAxis(3)) * -0.3;
    //         // }
    //         V_WheelRpmCmnd[index] =  (c_joyStick.GetRawAxis(2) - c_joyStick.GetRawAxis(3)) * -20;
          
    //       }
    //     }

    //   V_Mode = 1;
    //   }
    // else if ((c_joyStick.GetRawButton(5) == true) || (c_joyStick.GetRawButton(6) == true)) // Straif left 5, right 6
    //   {
    //   for (index = E_FrontLeft;
    //        index < E_RobotCornerSz;
    //        index = T_RobotCorner(int(index) + 1))
    //     {
    //     if (c_joyStick.GetRawButton(5) == true)
    //       {
    //       V_DesiredWheelAngle[index] = -90;
    //       }
    //     else
    //       {
    //       V_DesiredWheelAngle[index] = 90;
    //       }
    //     }

    //   if (V_Mode != 2)
    //     {
    //     V_WheelSpeedDelay = true;
    //     }

    //   if (V_WheelSpeedDelay == true)
    //     {
    //     L_WheelSpeedDelay = false;
    //     V_WheelSpeedDelay = false;

    //     for (index = E_FrontLeft;
    //          index < E_RobotCornerSz;
    //          index = T_RobotCorner(int(index) + 1))
    //       {
    //       L_WheelSpeedDelay =  CriteriaMet(V_DesiredWheelAngle[index],
    //                                        V_WheelAngle[index],
    //                                        5);
    //       if (L_WheelSpeedDelay == true)
    //         {
    //         V_WheelSpeedDelay = true;
    //         }
    //       }
    //     }

    //   for (index = E_FrontLeft;
    //        index < E_RobotCornerSz;
    //        index = T_RobotCorner(int(index) + 1))
    //     {
    //     if (V_WheelSpeedDelay == false)
    //       {
    //       V_WheelRpmCmnd[index] = 20;
    //       }
    //     else
    //       {
    //       V_WheelRpmCmnd[index] = 0.0;
    //       }
    //     }
    //   V_Mode = 2;
    //   }
    // else
    //   {
      
    //   double _JoyStickX = c_joyStick.GetRawAxis(0);
    //   double _JoyStickY = c_joyStick.GetRawAxis(1);
    //   double _JoyAngle;
    //   double _JoyStickZ = sqrt((_JoyStickX * _JoyStickX) + (_JoyStickY * _JoyStickY));
    //   if(_JoyStickY > 0.01) {
    //     _JoyAngle = RadtoDeg * atan(_JoyStickX/-_JoyStickY);  
    //   } else if(_JoyStickY < -0.01) {
    //     _JoyAngle = RadtoDeg * atan(-_JoyStickX/_JoyStickY);  
    //   } else {
    //     _JoyAngle = _JoyStickX * 90;
    //   }
       
    //    if (_JoyStickY < 0) {
    //      _JoyStickZ = -_JoyStickZ;
    //    }
    //    else if (_JoyStickY > 0) {
    //      _JoyStickZ = _JoyStickZ;
    //    }
      
      
      

    //   for (index = E_FrontLeft;
    //        index < E_RobotCornerSz;
    //        index = T_RobotCorner(int(index) + 1))
    //     {
        
    //     V_DesiredWheelAngle[index] = _JoyAngle;
    //     V_WheelRpmCmnd[index] = _JoyStickZ * -20;
    //     }
    //   V_WheelSpeedDelay = false;
    //   V_Mode = 3;
    //   }

    #pragma endregion

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
                                              0.0045, // P Gx
                                              0.0001, // I Gx
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
    //SmartDashboard
    #pragma region 

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

    #pragma endregion
  
    //Motor Set Functions
    #pragma region 
    // m_frontLeftDriveMotor.Set(V_WheelSpeedCmnd[E_FrontLeft]);
    // m_frontRightDriveMotor.Set(V_WheelSpeedCmnd[E_FrontRight]);
    // m_rearLeftDriveMotor.Set(V_WheelSpeedCmnd[E_RearLeft]);
    // m_rearRightDriveMotor.Set(V_WheelSpeedCmnd[E_RearRight]);

    // m_frontLeftDriveMotor.Set(0);
    // m_frontRightDriveMotor.Set(0);
    // m_rearLeftDriveMotor.Set(0);
    // m_rearRightDriveMotor.Set(0);

    m_frontLeftSteerMotor.Set(V_WheelAngleCmnd[E_FrontLeft] * (-1));
    m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight] * (-1));
    m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft] * (-1));
    m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight] * (-1));

    // m_frontLeftSteerMotor.Set (0);
    // m_frontRightSteerMotor.Set(0);
    // m_rearLeftSteerMotor.Set(0);
    // m_rearRightSteerMotor.Set(0);
    #pragma endregion

    // frc::Wait(0.01);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif