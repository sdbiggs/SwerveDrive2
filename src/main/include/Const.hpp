#include "Enums.hpp"
const double C_ExeTime = 0.01;
const double C_RadtoDeg = 57.2958;
const double C_Deg2Rad = 0.017453292519943295;
const double C_PI = 3.14159265358979;
const double C_Tau = 6.28318530717958647;

static const int frontLeftSteerDeviceID = 1, frontLeftDriveDeviceID = 2, frontRightSteerDeviceID = 4, frontRightDriveDeviceID = 3;
static const int rearLeftSteerDeviceID  = 5, rearLeftDriveDeviceID  = 6, rearRightSteerDeviceID  = 7, rearRightDriveDeviceID  = 8;
static const int topShooterID = 10, bottomShooterID = 9;
static const int liftID = 11;

const double K_ReductionRatio = 8.31;
const double K_WheelCircufrence = 12.566; // Circumferance of wheel, in inches

const double C_L = 0.5969;
const double C_W = 0.5969;
const double C_R = 0.8441;

const double K_ShooterWheelRotation[E_RoboShooter] = {5.12517590321455,     // E_TopShooter    2.5555555555555555555555555555555555555555555555 * 2 * C_PI * 0.3191858136047229930278045677412
                                                      3.84388192741092};    // E_BottomShooter 2.5555555555555555555555555555555555555555555555 * 2 * C_PI *0.2393893602035422447708534258059
<<<<<<< HEAD

const double K_WheelOfFortunePwr = 0.4; // This is the constant power command sent to the wheel of fortune spinner

const double K_InitAngle = 1.4; // This is the absolute angle that all of the wheels need to be sitting at before allowing the robot to exit init
const double K_WheelOffsetAngle[E_RobotCornerSz] = {75.234367,   // E_FrontLeft
                                                    90.615225,   // E_FrontRight 152  104.6
                                                    12.041014,   // E_RearLeft
                                                    180.703106}; // E_RearRight

const double K_WheelMaxSpeed = 200; // This is the max allowed speed for the wheels
=======
>>>>>>> 2b13db20198e29d8d628a3ab09832cb48193a05f

const double K_WheelAnglePID_Gx[E_PID_CalSz] = { 0.007,     // P Gx
                                                 0.0005,    // I Gx
                                                 0.0000005, // D Gx
                                                 0.4,       // P UL
                                                -0.4,       // P LL
                                                 0.12,      // I UL
                                                -0.12,      // I LL
                                                 0.5,       // D UL
                                                -0.5,       // D LL
                                                 0.9,       // Max upper
                                                -0.9};      // Max lower

<<<<<<< HEAD
const double K_WheelSpeedPID_Gx[E_PID_CalSz] = { 0.0055,     // P Gx
                                                 0.0009,     // I Gx
                                                 0.00000005, // D Gx
                                                 0.9,        // P UL
                                                -0.9,        // P LL
                                                 0.5,        // I UL
                                                -0.5,        // I LL
                                                 0.2,        // D UL
                                                -0.2,        // D LL
                                                 1.0,        // Max upper
                                                -1.0};       // Max lower

const double K_RobotRotationPID_Gx[E_PID_CalSz] = { 0.08,   // P Gx
                                                    0.0007, // I Gx
                                                    0.0,    // D Gx
                                                    0.9,    // P UL
                                                   -0.9,    // P LL
                                                    0.5,    // I UL
                                                   -0.5,    // I LL
                                                    0.2,    // D UL
                                                   -0.2,    // D LL
                                                    1.0,    // Max upper
                                                   -1.0};   // Max lower
=======
const double K_InitAngle = 1.4; // This is the absolute angle that all of the wheels need to be sitting at before allowing the robot to exit init
const double K_WheelOffsetAngle[E_RobotCornerSz] = {75.234367,   // E_FrontLeft
                                                    90.615225,   // E_FrontRight 152  104.6
                                                    12.041014,   // E_RearLeft
                                                    180.703106}; // E_RearRight
>>>>>>> 2b13db20198e29d8d628a3ab09832cb48193a05f

const double K_WheelMaxSpeed = 200; // This is the max allowed speed for the wheels

const double K_WheelAnglePID_Gx[E_PID_CalSz] = { 0.007,     // P Gx
                                                 0.0005,    // I Gx
                                                 0.0000005, // D Gx
                                                 0.4,       // P UL
                                                -0.4,       // P LL
                                                 0.12,      // I UL
                                                -0.12,      // I LL
                                                 0.5,       // D UL
                                                -0.5,       // D LL
                                                 0.9,       // Max upper
                                                -0.9};      // Max lower

const double K_WheelSpeedPID_Gx[E_PID_CalSz] = { 0.0055,     // P Gx
                                                 0.0009,     // I Gx
                                                 0.00000005, // D Gx
                                                 0.9,        // P UL
                                                -0.9,        // P LL
                                                 0.5,        // I UL
                                                -0.5,        // I LL
                                                 0.2,        // D UL
                                                -0.2,        // D LL
                                                 1.0,        // Max upper
                                                -1.0};       // Max lower

const double K_RobotRotationPID_Gx[E_PID_CalSz] = { 0.08,   // P Gx
                                                    0.0007, // I Gx
                                                    0.0,    // D Gx
                                                    0.9,    // P UL
                                                   -0.9,    // P LL
                                                    0.5,    // I UL
                                                   -0.5,    // I LL
                                                    0.2,    // D UL
                                                   -0.2,    // D LL
                                                    1.0,    // Max upper
                                                   -1.0};   // Max lower

const double K_DesiredDriveSpeedAxis[20] = {-0.95,
                                            -0.85,
                                            -0.75,
                                            -0.65,
                                            -0.55,
                                            -0.45,
                                            -0.35,
                                            -0.25,
                                            -0.15,
                                            -0.10,
                                             0.10,
                                             0.15,
                                             0.25,
                                             0.35,
                                             0.45,
                                             0.55,
                                             0.65,
                                             0.75,
                                             0.85,
                                             0.95};

const double K_DesiredDriveSpeed[20] = {-1.00,  //-0.95
                                        -0.88,  //-0.85
                                        -0.77,  //-0.75
                                        -0.66,  //-0.65
                                        -0.55,  //-0.55
                                        -0.44,  //-0.45
                                        -0.33,  //-0.35
                                        -0.22,  //-0.25
                                        -0.11,  //-0.15
                                         0.00,  //-0.10
                                         0.00,  // 0.10
                                         0.11,  // 0.15
                                         0.22,  // 0.25
                                         0.33,  // 0.35
                                         0.44,  // 0.45
                                         0.55,  // 0.55
                                         0.66,  // 0.65
                                         0.77,  // 0.75
                                         0.88,  // 0.85
                                         1.00}; // 0.95

<<<<<<< HEAD
const double K_TargetVisionAngle = 50;

=======
>>>>>>> 2b13db20198e29d8d628a3ab09832cb48193a05f
const double K_TargetVisionAngleMin = 10;

const double K_TargetVisionAngleMax = 50;

const double K_TargetVisionDistanceMin = 50;

const double K_TargetVisionDistanceMax = 50;

const double K_TargetVisionAngleErrorMax = 2;

const double K_TargetVisionUpperRollerErrorMax = 200;

const double K_TargetVisionLowerRollerErrorMax = 200;

#define K_BallLauncherDistanceSz 5
#define K_BallLauncherAngleSz 3

const double K_BallLauncherDistanceAxis[K_BallLauncherDistanceSz] = {400, 700, 968, 1300, 1660};

const double K_BallLauncherAngleAxis[K_BallLauncherAngleSz] = {-45, 0, 45};

const double K_BallLauncherRobotAngle[K_BallLauncherDistanceSz][K_BallLauncherAngleSz] =
  {
    {45, 0, -45},
    {45, 0, -45},
    {45, 0, -45},
    {45, 0, -45},
    {45, 0, -45}
  };

const double K_BallLauncherUpperSpeed[K_BallLauncherDistanceSz][K_BallLauncherAngleSz] =
  {
    {-1800, -1800, -1800},
    {-1800, -1800, -1800},
    {-1800, -1800, -1800},
    {-2000, -2000, -2000},
    {-2850, -2850, -2850}
  };

const double K_BallLauncherLowerSpeed[K_BallLauncherDistanceSz][K_BallLauncherAngleSz] =
  {
    {-2100, -2100, -2100},
    {-2100, -2100, -2100},
    {-2100, -2100, -2100},
    {-2500, -2500, -2500},
    {-2900, -2900, -2900}
  };
