#include "Enums.hpp"

T_AutoTargetStates AutoTargeting(T_AutoTargetStates  L_CurrentState,
                                 bool                L_Activate,
                                 double              L_DriverAxis1,
                                 double              L_DriverAxis2,
                                 double              L_DriverAxis3,
                                 double              L_RawTargetVisionAngle,
                                 double              L_RawTargetVisionDistance,
                                 double              L_RobotAngle,
                                 double             *L_RobotTargetAngle,
                                 double              L_UpperRollerSpeed,
                                 double              L_LowerRollerSpeed,
                                 double             *L_UpperRollerSpeedReq,
                                 double             *L_LowerRollerSpeedReq,
                                 double             *L_BeltPowerReq);
