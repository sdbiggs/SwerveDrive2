#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>


extern void visionInit(std::shared_ptr<NetworkTable> ntTable0,
                std::shared_ptr<NetworkTable> ntTable1,
                nt::NetworkTableInstance inst);
extern void visionOff(std::shared_ptr<NetworkTable> ntTable0,
               std::shared_ptr<NetworkTable> ntTable1,
               nt::NetworkTableInstance inst,
                bool ntStart1,
                bool ntStart2,
                bool ntVisionAngle,
                bool ntVisionDistance);
extern void visionRun(nt::NetworkTableEntry ntEntry,
              double ntDistance,
              int targetChoose,
              bool ntVisionAngle,
              bool ntVisionDistance,
              double ntDesiredAngle,
              double ntDesiredDistance);
extern double AutoTarget(double ntEntry);
extern double AutoShoot(nt::NetworkTableEntry targetYaw, 
                        double targetDistance, 
                        int TOPorBOTTOM);
extern bool AutoMove(double ntDistance, 
                     double distanceTOTAL);
