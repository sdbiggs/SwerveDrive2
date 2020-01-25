#include "Enums.hpp"

void Read_Encoders(bool L_RobotInit, double a_encoderFrontLeftSteerVoltage, double a_encoderFrontRightSteerVoltage,
    double a_encoderRearLeftSteerVoltage,double a_encoderRearRightSteerVoltage, rev::CANEncoder m_encoderFrontLeftSteer, rev::CANEncoder m_encoderFrontRightSteer,
    rev::CANEncoder m_encoderRearLeftSteer, rev::CANEncoder m_encoderRearRightSteer, rev::CANEncoder m_encoderFrontLeftDrive, rev::CANEncoder m_encoderFrontRightDrive, 
    rev::CANEncoder m_encoderRearLeftDrive, rev::CANEncoder m_encoderRearRightDrive);
 
 extern double V_WheelAngle[E_RobotCornerSz];
 extern double V_DesiredWheelAngle[E_RobotCornerSz];
 extern double V_WheelRelativeAngleRawOffset[E_RobotCornerSz];
 extern double V_WheelVelocity[E_RobotCornerSz];
