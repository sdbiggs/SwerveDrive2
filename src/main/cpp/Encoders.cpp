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
double V_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction
double V_WheelAngleRev[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a reverse direction
double V_WheelAngleArb[E_RobotCornerSz]; // This is the arbitrated wheel angle that is used in the PID controller
double V_WheelAnglePrev[E_RobotCornerSz];
double V_WheelAngleLoop[E_RobotCornerSz];
double V_WheelRelativeAngleRawOffset[E_RobotCornerSz];
double V_WheelVelocity[E_RobotCornerSz]; // Velocity of drive wheels, in in/sec

/******************************************************************************
 * Function:     Read_Encoders
 *
 * Description:  Run all of the encoder decoding logic.
 ******************************************************************************/
void Read_Encoders(bool            L_RobotInit,
                   double          a_encoderFrontLeftSteerVoltage,
                   double          a_encoderFrontRightSteerVoltage,
                   double          a_encoderRearLeftSteerVoltage,
                   double          a_encoderRearRightSteerVoltage,
                   rev::CANEncoder m_encoderFrontLeftSteer,
                   rev::CANEncoder m_encoderFrontRightSteer,
                   rev::CANEncoder m_encoderRearLeftSteer,
                   rev::CANEncoder m_encoderRearRightSteer,
                   rev::CANEncoder m_encoderFrontLeftDrive,
                   rev::CANEncoder m_encoderFrontRightDrive,
                   rev::CANEncoder m_encoderRearLeftDrive,
                   rev::CANEncoder m_encoderRearRightDrive)
  {
  T_RobotCorner index;

  if (L_RobotInit == true)
    {
    V_WheelAngleRaw[E_FrontLeft]  = a_encoderFrontLeftSteerVoltage * 72 - K_WheelOffsetAngle[E_FrontLeft];
    V_WheelAngleRaw[E_FrontRight] = a_encoderFrontRightSteerVoltage * 72 - K_WheelOffsetAngle[E_FrontRight];
    V_WheelAngleRaw[E_RearLeft]   = a_encoderRearLeftSteerVoltage * 72 - K_WheelOffsetAngle[E_RearLeft];
    V_WheelAngleRaw[E_RearRight]  = a_encoderRearRightSteerVoltage * 72 - K_WheelOffsetAngle[E_RearRight];

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
          V_WheelAngleFwd[index] = (V_WheelAngleLoop[index] * 360) + V_WheelAngleRaw[index];

          V_WheelAnglePrev[index] = V_WheelAngleRaw[index];
        }
    }
  else
    {
    V_WheelAngleFwd[E_FrontLeft]  = fmod(((m_encoderFrontLeftSteer.GetPosition()  - V_WheelRelativeAngleRawOffset[E_FrontLeft])  * -20), 360);
    V_WheelAngleFwd[E_FrontRight] = fmod(((m_encoderFrontRightSteer.GetPosition() - V_WheelRelativeAngleRawOffset[E_FrontRight]) * -20), 360);
    V_WheelAngleFwd[E_RearLeft]   = fmod(((m_encoderRearLeftSteer.GetPosition()   - V_WheelRelativeAngleRawOffset[E_RearLeft])   * -20), 360);
    V_WheelAngleFwd[E_RearRight]  = fmod(((m_encoderRearRightSteer.GetPosition()  - V_WheelRelativeAngleRawOffset[E_RearRight])  * -20), 360);

    for (index = E_FrontLeft;
         index < E_RobotCornerSz;
         index = T_RobotCorner(int(index) + 1))
      {
      if (V_WheelAngleFwd[index] > 180)
        {
        V_WheelAngleFwd[index] -= 360;
        }
      else if (V_WheelAngleFwd[index] < -180)
        {
        V_WheelAngleFwd[index] += 360;
        }

      /* Now we need to find the equivalent angle as if the wheel were going to be driven in the opposite direction, i.e. in reverse */
      if (V_WheelAngleFwd[index] >= 0)
        {
        V_WheelAngleRev[index] = V_WheelAngleFwd[index] - 180;
        }
      else
        {
        V_WheelAngleRev[index] = V_WheelAngleFwd[index] + 180;
        }
      }
    }

  V_WheelVelocity[E_FrontLeft]  = ((m_encoderFrontLeftDrive.GetVelocity()  / reductionRatio) / 60) * WheelCircufrence;
  V_WheelVelocity[E_FrontRight] = ((m_encoderFrontRightDrive.GetVelocity() / reductionRatio) / 60) * WheelCircufrence;
  V_WheelVelocity[E_RearRight]  = ((m_encoderRearRightDrive.GetVelocity()  / reductionRatio) / 60) * WheelCircufrence;
  V_WheelVelocity[E_RearLeft]   = ((m_encoderRearLeftDrive.GetVelocity()   / reductionRatio) / 60) * WheelCircufrence;
  }

/******************************************************************************
 * Function:     DtrmnEncoderRelativeToCmnd
 *
 * Description:  tbd
 ******************************************************************************/
double DtrmnEncoderRelativeToCmnd(double          L_JoystickCmnd,
                                  double          L_EncoderReading)
  {
    double L_Opt1;
    double L_Opt2;
    double L_Opt3;
    double L_Output;

    L_Opt1 = fabs(L_JoystickCmnd - L_EncoderReading);
    L_Opt2 = fabs(L_JoystickCmnd - (L_EncoderReading + 360));
    L_Opt3 = fabs(L_JoystickCmnd - (L_EncoderReading - 360));

    if ((L_Opt1 < L_Opt2) && (L_Opt1 < L_Opt3))
      {
        L_Output = L_EncoderReading;
      }
    else if ((L_Opt2 < L_Opt1) && (L_Opt2 < L_Opt3))
      {
        L_Output = L_EncoderReading + 360;
      }
    else
      {
        L_Output = L_EncoderReading - 360;
      }
      
    return (L_Output);
  }