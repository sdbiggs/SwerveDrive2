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

const double K_WheelOfFortunePwr = 0.4; // This is the constant power command sent to the wheel of fortune spinner

const double K_InitAngle = 1.4; // This is the absolute angle that all of the wheels need to be sitting at before allowing the robot to exit init
const double K_WheelOffsetAngle[E_RobotCornerSz] = {75.234367,   // E_FrontLeft
                                                    90.615225,   // E_FrontRight 152  104.6 
                                                    12.041014,   // E_RearLeft
                                                    144.580063}; // E_RearRight 180.703106  144.580063

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

const double K_RobotRotationPID_Gx[E_PID_CalSz] = { 0.07,   // P Gx
                                                    0.0,   // I Gx
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

/*  Rotation calibrations */
/* K_DesiredRotateSpeedAxis - This is the effective command axis, function of error calculation, in degrees */
const double K_DesiredRotateSpeedAxis[10] = {-20.0,
                                              -4.0,
                                              -2.0,
                                              -1.0,
                                              -0.2,
                                               0.2,
                                               1.0,
                                               2.0,
                                               4.0,
                                              20.0};

/* K_DesiredRotateSpeed - This is the effective command, equivalent to the rotate joystick */
const double K_DesiredRotateSpeed[10] = {-0.5,  // -20.0
                                         -0.2,  //  -4.0
                                         -0.06,  //  -2.0
                                         -0.05,  //  -1.0
                                          0.0,  //  -0.2
                                          0.0,  //   0.2
                                          0.05,  //   1.0
                                          0.06,  //   2.0
                                          0.2,  //   4.0
                                          0.5}; //  20.0

/* K_DesiredAutoRotateSpeedAxis - This is the effective command axis, function of error calculation, in degrees */
const double K_DesiredAutoRotateSpeedAxis[10] = {-4.0,
                                              -3.0,
                                              -2.0,
                                              -1.0,
                                              -0.2,
                                               0.2,
                                               1.0,
                                               2.0,
                                               3.0,
                                              4.0};

/* K_DesiredRotateSpeed - This is the effective command, equivalent to the rotate joystick */
const double K_DesiredAutoRotateSpeed[10] = {-0.06,  // -4.0
                                         -0.05,  //  -3.0
                                         -0.04,  //  -2.0
                                         -0.03,  //  -1.0
                                         -0.01,  //  -0.2
                                          0.01,  //   0.2
                                          0.03,  //   1.0
                                          0.04,  //   2.0
                                          0.05,  //   4.0
                                          0.06}; //  20.0
const double K_DesiredDistanceAxis[6] = {415,
                                     644,
                                     840,
                                     975,
                                     1077,
                                     1667};

const double K_DesiredSpeedUpperBeam[6] = {-950,
                                           -950,
                                           -1185,
                                           -1585,
                                           -1715,
                                           -2785};
                                          //  {-1200,
                                          //  -1200,
                                          //  -1435,
                                          //  -1815,
                                          //  -1965,
                                          //  -3015};


const double K_DesiredSpeedLowerBeam[6] = {-1150,
                                          -1350,
                                          -1880,
                                          -2100,
                                          -2400,
                                          -3100};
// This is the amount of time that we will wait to make sure we are at the correct location
const double K_RotateDebounceTime = 0.06;  

// This is the amount of error allowed when in auto rotate / auto target
const double K_RotateDeadbandAngle = 0.420;  

// This is the desired target angle for the auto vision targeting.  This is due to the offset of the camera.
const double K_TargetVisionAngle = 3.3;




const double K_TargetVisionAngleMin = 10;

const double K_TargetVisionAngleMax = 50;

const double K_TargetVisionDistanceMin = 50;

const double K_TargetVisionDistanceMax = 50;

const double K_TargetVisionAngleErrorMax = 2;

const double K_TargetVisionUpperRollerErrorMax = 200;

const double K_TargetVisionLowerRollerErrorMax = 200;

const double K_RotateDebounceThreshold = 0.1;

const double K_MaxGain = 0.75;

const double K_AutoRotateGx = 0.1;

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



/* Auton specific cals */
#include "K_L_AutonX_Position.hpp"
#include "K_L_AutonY_Position.hpp"
#include "K_t_AutonXY_PositionAxis.hpp"

const double K_k_AutonX_PID_Gx[E_PID_CalSz] = { 0.5,       // P Gx
                                                0.0005,    // I Gx
                                                0.05,      // D Gx
                                                0.8,       // P UL
                                               -0.8,       // P LL
                                                0.12,      // I UL
                                               -0.12,      // I LL
                                                0.5,       // D UL
                                               -0.5,       // D LL
                                                0.9,       // Max upper
                                               -0.9};      // Max lower

const double K_k_AutonY_PID_Gx[E_PID_CalSz] = { 0.5,       // P Gx
                                                0.0005,    // I Gx
                                                0.05,      // D Gx
                                                0.8,       // P UL
                                               -0.8,       // P LL
                                                0.12,      // I UL
                                               -0.12,      // I LL
                                                0.5,       // D UL
                                               -0.5,       // D LL
                                                0.9,       // Max upper
                                               -0.9};      // Max lower