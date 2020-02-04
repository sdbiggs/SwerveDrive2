#include "Enums.hpp"

static const int frontLeftSteerDeviceID = 1, frontLeftDriveDeviceID = 2, frontRightSteerDeviceID = 4, frontRightDriveDeviceID = 3;
static const int rearLeftSteerDeviceID  = 5, rearLeftDriveDeviceID  = 6, rearRightSteerDeviceID  = 7, rearRightDriveDeviceID  = 8;
static const int topShooterID = 9, bottomShooterID = 10;
const double reductionRatio = 8.31;
const int WheelDiamter = 4;
const double WheelCircufrence = 12.566; // Circumferance of wheel, in inches
const double RadtoDeg = 57.2958;
const double C_ExeTime = 0.01;
const double C_Lenght = 0.5969;
const double C_Width = 0.5969;
const double C_R = 0.8441;
const double C_PI = 3.14159265358979;
const double K_InitAngle = 1.4; // This is the absolute angle that all of the wheels need to be sitting at before allowing the robot to exit init
const double K_WheelMaxSpeed = 200; // This is the max allowed speed for the wheels

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
