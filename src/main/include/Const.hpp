static const int frontLeftSteerDeviceID = 1, frontLeftDriveDeviceID = 2, frontRightSteerDeviceID = 4, frontRightDriveDeviceID = 3;
static const int rearLeftSteerDeviceID  = 5, rearLeftDriveDeviceID  = 6, rearRightSteerDeviceID  = 7, rearRightDriveDeviceID  = 8;
const double reductionRatio = 8.31;
const int WheelDiamter = 4;
const double WheelCircufrence = 12.566;
const double RadtoDeg = 57.2958;
const double C_ExeTime = 0.01;
const double C_Lenght = 0.5969;
const double C_Width = 0.5969;
const double C_R = 0.8441;
const double C_PI = 3.14159265358979;

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