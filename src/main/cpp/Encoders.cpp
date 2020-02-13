/*
  Encoders.cpp

  Created on: Jan 3, 2020
  Author: 5561
 */

#include "rev/CANSparkMax.h"
#include <frc/AnalogInput.h>
#include "Enums.hpp"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Encoders.hpp"
#include "Const.hpp"

double V_WheelAngleRaw[E_RobotCornerSz];
 double V_WheelAngle[E_RobotCornerSz];
 double V_WheelAnglePrev[E_RobotCornerSz];
 double V_WheelAngleLoop[E_RobotCornerSz];
 double V_DesiredWheelAngle[E_RobotCornerSz];
 double V_WheelRelativeAngleRawOffset[E_RobotCornerSz];
 double V_WheelVelocity[E_RobotCornerSz];

/******************************************************************************
 * Function:     Read_Encoders
 *
 * Description:  Run all of the encoder decoding logic.
 ******************************************************************************/
void Read_Encoders(bool L_RobotInit, 
    double a_encoderFrontLeftSteerVoltage, double a_encoderFrontRightSteerVoltage,double a_encoderRearLeftSteerVoltage,double a_encoderRearRightSteerVoltage, 
    rev::CANEncoder m_encoderFrontLeftSteer, rev::CANEncoder m_encoderFrontRightSteer,rev::CANEncoder m_encoderRearLeftSteer, rev::CANEncoder m_encoderRearRightSteer, 
    rev::CANEncoder m_encoderFrontLeftDrive, rev::CANEncoder m_encoderFrontRightDrive, rev::CANEncoder m_encoderRearLeftDrive, rev::CANEncoder m_encoderRearRightDrive)
{
  double l_FL_Angle = 0;

T_RobotCorner index;

  if (L_RobotInit == true)
  {
    V_WheelAngleRaw[E_FrontLeft]  = a_encoderFrontLeftSteerVoltage * 72 - 322;
    V_WheelAngleRaw[E_FrontRight] = a_encoderFrontRightSteerVoltage * 72 - 152;
    V_WheelAngleRaw[E_RearLeft]   = a_encoderRearLeftSteerVoltage * 72 - 14;
    V_WheelAngleRaw[E_RearRight]  = a_encoderRearRightSteerVoltage * 72 - 180.6;

    V_WheelRelativeAngleRawOffset[E_FrontLeft] = m_encoderFrontLeftSteer.GetPosition();
    V_WheelRelativeAngleRawOffset[E_FrontRight] = m_encoderFrontRightSteer.GetPosition();
    V_WheelRelativeAngleRawOffset[E_RearLeft] = m_encoderRearLeftSteer.GetPosition();
    V_WheelRelativeAngleRawOffset[E_RearRight] = m_encoderRearRightSteer.GetPosition();

      for (index = E_FrontLeft;
           index < E_RobotCornerSz;
           index = T_RobotCorner(int(index) + 1))
        {
        if(abs(V_WheelAnglePrev[index]) >= 330 || abs(V_WheelAnglePrev[index] <= 30 ))
         {
          if(V_WheelAnglePrev[index] >= 330 && V_WheelAngleRaw[index] <= 30)
           {
            V_WheelAngleLoop[index] += 1;
            }
          else if (V_WheelAnglePrev[index] <= 30 && V_WheelAngleRaw[index] >= 330)
           {
            V_WheelAngleLoop[index] -= 1;
           }
          }
          V_WheelAngle[index] = (V_WheelAngleLoop[index] * 360) + V_WheelAngleRaw[index];

          V_WheelAnglePrev[index] = V_WheelAngleRaw[index];
        }

    V_DesiredWheelAngle[E_FrontLeft] = 0;
    V_DesiredWheelAngle[E_FrontRight] = 0;
    V_DesiredWheelAngle[E_RearLeft] = 0;
    V_DesiredWheelAngle[E_RearRight] = 0;
  }else
  {
  V_WheelAngle[E_FrontLeft] = fmod(((m_encoderFrontLeftSteer.GetPosition() - V_WheelRelativeAngleRawOffset[E_FrontLeft]) * -20), 360);
  V_WheelAngle[E_FrontRight] = fmod(((m_encoderFrontRightSteer.GetPosition() - V_WheelRelativeAngleRawOffset[E_FrontRight]) * -20), 360);
  V_WheelAngle[E_RearLeft] = fmod(((m_encoderRearLeftSteer.GetPosition() - V_WheelRelativeAngleRawOffset[E_RearLeft]) * -20), 360);
  V_WheelAngle[E_RearRight] = fmod(((m_encoderRearRightSteer.GetPosition() - V_WheelRelativeAngleRawOffset[E_RearRight]) * -20), 360);
  }
    
  V_WheelVelocity[E_FrontLeft] = ((m_encoderFrontLeftDrive.GetVelocity() / reductionRatio) / 60) * WheelCircufrence;
  V_WheelVelocity[E_FrontRight] =((m_encoderFrontRightDrive.GetVelocity() / reductionRatio) / 60) * WheelCircufrence;
  V_WheelVelocity[E_RearRight] = ((m_encoderRearRightDrive.GetVelocity() / reductionRatio) / 60) * WheelCircufrence;
  V_WheelVelocity[E_RearLeft] = ((m_encoderRearLeftDrive.GetVelocity() / reductionRatio) / 60) * WheelCircufrence;

frc::SmartDashboard::PutNumber("Raw FL", V_WheelAngleLoop[E_FrontLeft]);
frc::SmartDashboard::PutNumber("Raw FR", V_WheelAngleLoop[E_FrontRight]);
frc::SmartDashboard::PutNumber("Raw RL", V_WheelAngleLoop[E_RearLeft]);
frc::SmartDashboard::PutNumber("Raw RR", V_WheelAngleLoop[E_RearRight]);


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

  frc::SmartDashboard::PutNumber("Encoder Front Left", a_encoderFrontLeftSteerVoltage);
  frc::SmartDashboard::PutNumber("Encoder Front Right", a_encoderFrontRightSteerVoltage);
  frc::SmartDashboard::PutNumber("Encoder Rear Left", a_encoderRearLeftSteerVoltage);
  frc::SmartDashboard::PutNumber("Encoder Rear Right", a_encoderRearRightSteerVoltage);

  frc::SmartDashboard::PutNumber("Front Left Wheel Position",m_encoderFrontLeftDrive.GetPosition() / reductionRatio);
  frc::SmartDashboard::PutNumber("Front Right Wheel Position",m_encoderFrontRightDrive.GetPosition() / reductionRatio);
  frc::SmartDashboard::PutNumber("Rear Left Wheel Position",m_encoderRearLeftDrive.GetPosition() / reductionRatio);
  frc::SmartDashboard::PutNumber("Rear Right Wheel Position",m_encoderRearRightDrive.GetPosition() / reductionRatio);

  }
