//pls ignore this file


// #include "Robot.h"
// #include "control_pid.hpp"
// #include "Encoders.hpp"
// #include "DriveControl.hpp"
// #include "Lookup.hpp"
// #include "Enums.hpp"
// #include <math.h>
// #include <Robot.h>
// #include "Robot.cpp"



// using namespace frc;
// //defining variable we need
// T_AutoTargetStates AutoTargeting(T_AutoTargetStates  L_CurrentState,
//                                  bool                L_Activate,
//                                  double              L_DriverAxis1,
//                                  double              L_DriverAxis2,
//                                  double              L_DriverAxis3,
//                                  double              L_RawTargetVisionAngle,
//                                  double              L_RawTargetVisionDistance,
//                                  double              L_RobotAngle,
//                                  double             *L_RobotTargetAngle,
//                                  double              L_UpperRollerSpeed,
//                                  double              L_LowerRollerSpeed,
//                                  double             *L_UpperRollerSpeedReq,
//                                  double             *L_LowerRollerSpeedReq,
//                                  double             *L_BeltPowerReq)
// {  
//   //shutting off roller acceleration and setting AbortTargeting to false
//   double L_UpperRollerSpeedError = 0;
//   double L_LowerRollerSpeedError = 0;
//   double L_RobotAngleError  = 0;
//   bool   L_AbortTargeting   = false;

//   //set up our aborting conditions
//   if ((fabs(L_DriverAxis1) > 0) ||
//       (fabs(L_DriverAxis2) > 0) ||
//       (fabs(L_DriverAxis3) > 0))
//     {
//     L_AbortTargeting = true;
//     }
//   //define errors
//   L_RobotAngleError       = fabs(L_RobotAngle       - *L_RobotTargetAngle);
//   L_UpperRollerSpeedError = fabs(L_UpperRollerSpeed - *L_UpperRollerSpeedReq);
//   L_LowerRollerSpeedError = fabs(L_LowerRollerSpeed - *L_LowerRollerSpeedReq);
//   //abort conditions
//   if (L_AbortTargeting == true)
//     {
//     L_CurrentState = E_NotActive;
//     *L_RobotTargetAngle = 0.0;
//     *L_UpperRollerSpeedReq = 0.0;
//     *L_LowerRollerSpeedReq = 0.0;
//     *L_BeltPowerReq = 0.0;
//     }
// }

// int MoveIntoPos(driveforward,
//                   0,
//                   0,
//                   c_joyStick.GetRawAxis(3),
//                   c_joyStick.GetRawButton(1),
//                   c_joyStick.GetRawButton(3),
//                   c_joyStick.GetRawButton(4),
//                   c_joyStick.GetRawButton(5),
//                   gyro_yawangledegrees,
//                   gyro_yawanglerad,
//                   targetYaw0.GetDouble(0),
//                   &V_WheelAngleFwd[0],
//                   &V_WheelAngleRev[0],
//                   &V_WS[0],
//                   &V_WA[0],
//                   &V_RobotInit,
//                   V_AutoTargetState,
//                   V_AutoTargetAngle)