#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>

// std::shared_ptr<NetworkTable> vision0;
// std::shared_ptr<NetworkTable> vision1;
// std::shared_ptr<NetworkTable> lidar;
// std::shared_ptr<NetworkTable> ledLight;

// nt::NetworkTableInstance inst;
// nt::NetworkTableEntry driverMode0;
// nt::NetworkTableEntry targetYaw0;
// nt::NetworkTableEntry targetPitch0;
// nt::NetworkTableEntry targetPose0;
// nt::NetworkTableEntry latency0;
// nt::NetworkTableEntry driverMode1;
// nt::NetworkTableEntry targetYaw1;
// nt::NetworkTableEntry targetPitch1;
// nt::NetworkTableEntry targetPose1;
// nt::NetworkTableEntry latency1;
// nt::NetworkTableEntry lidarDistance;
// nt::NetworkTableEntry ledControl;

// double       distanceTarget;
// double       distanceBall;
// double       distanceFromTargetCenter;
// double       distanceFromBallCenter;
// double       desiredVisionAngle0;
// double       desiredVisionDistance0;
// bool         activeVisionAngle0;
// bool         activeVisionDistance0;
// bool         visionRequest;
// bool         visionStart[2] {false, false};
// enum         VisionAuton {strafe, rotate, complete};
// enum         VisionTarget{high, ball};

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
extern double AutoTarget(nt::NetworkTableEntry ntEntry,
              double ntDistance,
              int targetChoose,
              bool ntVisionAngle,
              bool ntVisionDistance,
              double ntDesiredAngle,
              double ntDesiredDistance);
// extern double AutoShoot(nt::NetworkTableEntry targetYaw, 
//                         double targetDistance, 
//                         int TOPorBOTTOM);