#include "vision.hpp"

/******************************************************************************
 * Function:     visionInit
 *
 * Description:  This function sets up camera and lights.
 ******************************************************************************/
void visionInit(std::shared_ptr<NetworkTable> ntTable0,
                std::shared_ptr<NetworkTable> ntTable1,
                nt::NetworkTableInstance inst)
{
    ntTable1->PutNumber("ledControl", 1);
    ntTable0->PutBoolean("driverMode", false);
    inst.Flush();
}

/******************************************************************************
 * Function:     visionOff
 *
 * Description:  This function turns on driver mode and turns off lights.
 ******************************************************************************/
void visionOff(std::shared_ptr<NetworkTable> ntTable0,
                std::shared_ptr<NetworkTable> ntTable1,
                nt::NetworkTableInstance inst,
                bool ntStart1,
                bool ntStart2,
                bool ntVisionAngle,
                bool ntVisionDistance)
{
    ntTable1->PutNumber("ledControl", 5);
    ntTable0->PutBoolean("driverMode", true);
    inst.Flush();
    ntStart1 = false; ntStart2 = false;
    ntVisionAngle = false;
    ntVisionDistance = false;
}
    
/******************************************************************************
 * Function:     visionRun
 *
 * Description:  This function toggles vision loop.
 ******************************************************************************/
void visionRun(nt::NetworkTableEntry ntEntry,
              double ntDistance,
              int targetChoose,
              bool ntVisionAngle,
              bool ntVisionDistance,
              double ntDesiredAngle,
              double ntDesiredDistance)
{
    int autonChoose;
    
      switch (targetChoose)
      {
          case 0:
            if (abs(ntEntry.GetDouble(0)) > 1)
            {
                autonChoose = 1;
            }
            if (abs(ntEntry.GetDouble(0)) < 1)
            {
                autonChoose = 2;
            }
            break;

          case 1:
            if (abs(ntEntry.GetDouble(0)) > 5)
            {
                autonChoose = 0;
            }
            else if (abs(ntEntry.GetDouble(0)) > 1)
            {
                ntVisionDistance = false;
                autonChoose = 1;
            }
            if (abs(ntEntry.GetDouble(0)) < 1)
            {
                autonChoose = 2;
            }
            break;
      }
      
      switch (autonChoose)
      {
          case 0:
            if (ntVisionDistance == false)
            {
                ntDesiredDistance = floor(ntDistance);
                ntVisionDistance  = true;
            }
            // if (visionRequest == true)
            // {
            //     visionRequest = false;
            //     activeVisionDistance0 = false;
            // }
            break;
          
          case 1:
            if (ntVisionAngle == false)
            {
                ntDesiredAngle = (0.9 * ntEntry.GetDouble(0));
                ntVisionAngle  = true;
            }
            // if (visionRequest == true)
            // {
            //     visionRequest = false;
            //     activeVisionDistance0 = false;
            // }
            break;
          
          case 2:
            break;
      }
}
