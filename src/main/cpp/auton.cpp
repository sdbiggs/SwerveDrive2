#include "vision.hpp"
#include "Gyro.hpp"
#include "Robot.h"
#include <iostream>

using namespace frc;

double AutoShoot(nt::NetworkTableEntry targetYaw, double targetDistance, int TOPorBOTTOM)
{
    // double speedDesired = LookUpForTOPorBOTTOM(targetYaw, targetDistance, TOPorBOTTOM);
    double speedDesired;
    return speedDesired;
}

double AutoTarget(double ntEntry)
{
    double rotate = 0;
    bool ntVisionAngle = false;
    double ntDesiredAngle = 0;
    
    if (abs(ntEntry) > 1)
    {
        ntDesiredAngle = (0.9 * ntEntry);
        ntVisionAngle  = true;
    }
    else
    {
        ntVisionAngle = false;
        rotate = 0;
    }
    
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

bool AutoMove(double ntDistance, double distanceTOTAL)
{
    bool positionAchieved;
    double displacement = (distanceTOTAL - ntDistance);

    if(displacement <= 0)
    {
        positionAchieved = true;
    }

    return positionAchieved;
}