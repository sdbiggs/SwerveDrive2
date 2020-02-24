#include "Enums.hpp"

static const int frontLeftSteerDeviceID = 1, frontLeftDriveDeviceID = 2, frontRightSteerDeviceID = 4, frontRightDriveDeviceID = 3;
static const int rearLeftSteerDeviceID  = 5, rearLeftDriveDeviceID  = 6, rearRightSteerDeviceID  = 7, rearRightDriveDeviceID  = 8;
static const int topShooterID = 10, bottomShooterID = 9;
static const int liftID = 11;
const double reductionRatio = 8.31;
const int WheelDiamter = 4;
const double WheelCircufrence = 12.566; // Circumferance of wheel, in inches
const double RadtoDeg = 57.2958;
const double C_ExeTime = 0.01;
const double C_L = 0.5969;
const double C_W = 0.5969;
const double C_R = 0.8441;
const double C_PI = 3.14159265358979;
const double K_InitAngle = 1.4; // This is the absolute angle that all of the wheels need to be sitting at before allowing the robot to exit init
const double K_WheelMaxSpeed = 200; // This is the max allowed speed for the wheels
const double deg2rad = 0.017453292519943295;
const double tau = 6.28318530717958647;

const double K_WheelOfFortunePwr = 0.1; // This is the constant power command sent to the wheel of fortune spinner

const double K_WheelOffsetAngle[E_RobotCornerSz] = {75.234367,    // E_FrontLeft
                                                    90.615225,    // E_FrontRight 152  104.6
                                                     12.041014,    // E_RearLeft
                                                    180.703106}; // E_RearRight

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

const double K_DesiredDriveSpeed[20] = {-1,  //-0.95
                                        -0.88,  //-0.85
                                         -0.77, //-0.75
                                         -0.66,//-0.65
                                         -0.55,  //-0.55
                                         -0.44, //-0.45
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
                                         1}; // 0.95


#define K_BallLauncherDistanceSz 5
#define K_BallLauncherAngleSz 3

const double K_BallLauncherDistanceAxis[K_BallLauncherDistanceSz] = {5, 10, 15, 20, 25};

const double K_BallLauncherAngleAxis[K_BallLauncherAngleSz] = {-45, 0, 45};

const double K_BallLauncherUpperSpeed[K_BallLauncherDistanceSz][K_BallLauncherAngleSz] =
  {
    {2000, 2100, 2200},
    {2300, 2400, 2500},
    {2600, 2700, 2800},
    {2900, 3100, 3200},
    {3300, 3400, 3500}
  };

const double K_BallLauncherLowerSpeed[K_BallLauncherDistanceSz][K_BallLauncherAngleSz] =
  {
    {2000, 2100, 2200},
    {2300, 2400, 2500},
    {2600, 2700, 2800},
    {2900, 3100, 3200},
    {3300, 3400, 3500}
  };