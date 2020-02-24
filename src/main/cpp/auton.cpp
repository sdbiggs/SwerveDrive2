#include "vision.hpp"
#include "Gyro.hpp"
#include "Robot.h"
#include <iostream>

using namespace frc;

// double AutoShoot(nt::NetworkTableEntry targetYaw, double targetDistance, int TOPorBOTTOM)
// {
//     // double speedDesired = LookUpForTOPorBOTTOM(targetYaw, targetDistance, TOPorBOTTOM);
//     // return speedDesired;
// }

double AutoTarget(nt::NetworkTableEntry ntEntry,
              double ntDistance,
              int targetChoose,
              bool ntVisionAngle,
              bool ntVisionDistance,
              double ntDesiredAngle,
              double ntDesiredDistance)
{
    double rotate = 0;
    visionRun(ntEntry, ntDistance, targetChoose, ntVisionAngle, ntVisionDistance, ntDesiredAngle, ntDesiredDistance);
    
    if(ntVisionAngle == true)
    {
        if(ntDesiredAngle < 0)
        {
            rotate = 1;
        }
        if(ntDesiredAngle > 0)
        {
            rotate = -1;
        }
    }

    return rotate;
}

// bool AutoLongitudinalMove(double ntDistance, double distanceTOTAL)
// {
//     bool longitudinalAchieved;
//     double displacement = (distanceTOTAL - ntDistance);

//     if(displacement <= 0)
//     {
//         longitudinalAchieved = true;
//     }

//     return longitudinalAchieved;
// }

// bool AutoLateralMove(double distanceFROMCENTER, double distanceTOTAL)
// {
//     bool lateralAchieved;
//     double displacement = (distanceTOTAL - distanceFROMCENTER);
// }