#include <math.h>

#include "Lookup.hpp"
#include "Gyro.hpp"
#include "Const.hpp"
#include "Enums.hpp"

double WheelSpeed[E_RobotCornerSz];
double WheelAngle[E_RobotCornerSz];
double WheelAnglePrev[E_RobotCornerSz];
double WheelAngleLoopcount[E_RobotCornerSz];
double WheelAngleFinal[E_RobotCornerSz];
double WheelAngleCase[E_RobotCornerSz];

void SwerveDriveWheelOutput(double Joystick_Forward, double Joystick_Strafe, double Joystick_Rotate, double Joystick_Gain, double *CurrentWheelAngle, double *L_WA, double *L_WS)
{
    Joystick_Forward = DesiredSpeed(Joystick_Forward);
    Joystick_Strafe = DesiredSpeed(Joystick_Strafe);
    Joystick_Rotate = DesiredSpeed(Joystick_Rotate);

    double L_temp = Joystick_Forward * cos(gyro_yawanglerad) + Joystick_Strafe * sin(gyro_yawanglerad);
    Joystick_Strafe = -Joystick_Forward * sin(gyro_yawanglerad) + Joystick_Strafe * cos(gyro_yawanglerad);
    Joystick_Forward = L_temp;

    double V_A = Joystick_Strafe - Joystick_Rotate * (C_L/C_R);
    double V_B = Joystick_Strafe + Joystick_Rotate * (C_L/C_R);
    double V_C = Joystick_Forward - Joystick_Rotate * (C_W/C_R);
    double V_D = Joystick_Forward + Joystick_Rotate * (C_W/C_R);   

    WheelSpeed[E_FrontRight] = pow((V_B * V_B + V_C * V_C), 0.5);
    WheelSpeed[E_FrontLeft] = pow((V_B * V_B + V_D * V_D), 0.5);
    WheelSpeed[E_RearLeft] = pow((V_A * V_A + V_D * V_D), 0.5);
    WheelSpeed[E_RearRight] = pow((V_A * V_A + V_C * V_C), 0.5);

    T_RobotCorner index = E_FrontLeft;
    for (index = E_FrontLeft; index < E_RobotCornerSz; index = T_RobotCorner(int(index) + 1)){
        WheelAnglePrev[index] = WheelAngle[index];
    }

    WheelAngle[E_FrontRight] = atan2(V_B, V_C) *180/C_PI;
    WheelAngle[E_FrontLeft] = atan2(V_B, V_D) *180/C_PI;
    WheelAngle[E_RearLeft] = atan2(V_A, V_D) *180/C_PI;
    WheelAngle[E_RearRight] = atan2(V_A, V_C) *180/C_PI;

    for (index = E_FrontLeft; index < E_RobotCornerSz; index = T_RobotCorner(int(index) + 1))
    {
    if(abs(WheelAnglePrev[index]) >= 150)
      {
        if(WheelAnglePrev[index] < 0 && WheelAngle[index] > 0)
          {
           WheelAngleLoopcount[index] += 1;
          }     
        else if (WheelAnglePrev[index] > 0 && WheelAngle[index] < 0)
          {
           WheelAngleLoopcount[index] -= 1;
          }
     }
    WheelAngleFinal[index] = (WheelAngleLoopcount[index] * 360) + WheelAngle[index];
    //V_WA_Final[index] = V_WA[index];
    }

    double V_Max = WheelSpeed[E_FrontRight];

    if (WheelSpeed[E_FrontLeft] > V_Max) {
        V_Max = WheelSpeed[E_FrontLeft];
    }
    if (WheelSpeed[E_RearLeft] > V_Max) {
        V_Max = WheelSpeed[E_RearLeft];
    }
    if (WheelSpeed[E_RearRight] > V_Max) {
        V_Max = WheelSpeed[E_RearRight];
    }
    if (V_Max > 1) {
        WheelSpeed[E_FrontRight] /= V_Max;
        WheelSpeed[E_FrontLeft] /= V_Max;
        WheelSpeed[E_RearLeft] /= V_Max;
        WheelSpeed[E_RearRight] /= V_Max;
    }

    double L_Gain;
    if (Joystick_Gain < 0.1){
        L_Gain = 0.1;
    }
    else{
        L_Gain = Joystick_Gain;
    }

    WheelSpeed[E_FrontRight] *= (150 * L_Gain);
    WheelSpeed[E_FrontLeft] *= (150 * L_Gain);
    WheelSpeed[E_RearLeft] *= (150 * L_Gain);
    WheelSpeed[E_RearRight] *= (150 * L_Gain);

    double opt1;
    double opt2;

    for (index = E_FrontLeft; index< E_RobotCornerSz; index = T_RobotCorner(int(index) + 1)) {
        WheelAngleCase[index] = 0;
        opt1 = abs(CurrentWheelAngle[index] - WheelAngle[index]);
        opt2 = abs(CurrentWheelAngle[index] - (WheelAngle[index] + 180));

        if (opt1 > 180 && opt2 > 180){
            if (opt1 < opt2){
                WheelAngle[index] -= 360;
                WheelAngleCase[index] = 1;
            } else {
                WheelAngle[index] += 360;
                WheelSpeed[index] *= -1;
                WheelAngleCase[index] = 2;
            }
        } else {
            if (opt1 < opt2){
                WheelAngleCase[index] = 3;
            } else {
                WheelAngle[index] += 180;
                WheelSpeed[index] *= -1;
                WheelAngleCase[index] = 4;
            }
        }
    }

    for (index = E_FrontLeft; index< E_RobotCornerSz; index = T_RobotCorner(int(index) + 1)) {
        L_WA[index] = WheelAngle[index];
        L_WS[index] = WheelSpeed[index];
    }
}
