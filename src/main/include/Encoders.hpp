#include "Enums.hpp"

void Read_Encoders(bool L_RobotInit,
                   double a_encoderFrontLeftSteerVoltage,
                   double a_encoderFrontRightSteerVoltage,
                   double a_encoderRearLeftSteerVoltage,
                   double a_encoderRearRightSteerVoltage,
                   rev::CANEncoder m_encoderFrontLeftSteer,
                   rev::CANEncoder m_encoderFrontRightSteer,
                   rev::CANEncoder m_encoderRearLeftSteer,
                   rev::CANEncoder m_encoderRearRightSteer,
                   rev::CANEncoder m_encoderFrontLeftDrive,
                   rev::CANEncoder m_encoderFrontRightDrive,
                   rev::CANEncoder m_encoderRearLeftDrive,
                   rev::CANEncoder m_encoderRearRightDrive,
                   rev::CANEncoder m_encoderTopShooter,
                   rev::CANEncoder m_encoderBottomShooter);

double DtrmnEncoderRelativeToCmnd(double L_JoystickCmnd,
                                  double L_EncoderReading);

 extern double V_WheelAngleFwd[E_RobotCornerSz];
 extern double V_Rad_WheelAngleFwd[E_RobotCornerSz]; 
 extern double V_WheelAngleRev[E_RobotCornerSz];
 extern double V_WheelAngleArb[E_RobotCornerSz];
 extern double V_WheelRelativeAngleRawOffset[E_RobotCornerSz];
 extern double V_WheelVelocity[E_RobotCornerSz];
 extern double V_WheelAnglePrev[E_RobotCornerSz];
 extern double V_WheelAngleLoop[E_RobotCornerSz];
 extern double V_WheelAngleRaw[E_RobotCornerSz];
 extern double V_ShooterSpeedCurr[E_RoboShooter];
 extern double V_WheelDeltaDistance[E_RobotCornerSz];
 extern double V_M_WheelDeltaDistance[E_RobotCornerSz];
 extern double V_Cnt_WheelDeltaDistanceCurr[E_RobotCornerSz];
 extern double V_Cnt_WheelDeltaDistancePrev[E_RobotCornerSz];
 