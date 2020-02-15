#include <math.h>
#include "Robot.h"
#include "Lookup.hpp"
#include "Gyro.hpp"
#include "Encoders.hpp"

double WheelSpeed[E_RobotCornerSz];
double WheelAngle[E_RobotCornerSz];
double WheelAnglePrev[E_RobotCornerSz];
double WheelAngleLoopcount[E_RobotCornerSz];
double WheelAngleFinal[E_RobotCornerSz];
double WheelAngleCase[E_RobotCornerSz];

void SwerveDriveWheelOutput(double L_FWD,
                            double L_STR,
                            double L_RCW,
                            double Joystick_Gain,
                            double *CurrentWheelAngle,
                            double *L_WA,
                            double *L_WS)
{
  double L_temp;
  double L_A;
  double L_B;
  double L_C;
  double L_D;
  double L_Max;
  double L_Gain;
  double L_WA_FWD;
  double L_WA_REV;
  T_RobotCorner index;
  double L_WA_FWD_Delta;
  double L_WA_REV_Delta;



  L_temp = L_FWD * cos(gyro_yawanglerad) + L_STR * sin(gyro_yawanglerad);
  L_STR = -L_FWD * sin(gyro_yawanglerad) + L_STR * cos(gyro_yawanglerad);
  L_FWD = L_temp;

  //Ws1: fr, Ws2: fl, ws3: rl, ws4: rr
  L_A = L_STR - L_RCW * (C_L/C_R);
  L_B = L_STR + L_RCW * (C_L/C_R);
  L_C = L_FWD - L_RCW * (C_W/C_R);
  L_D = L_FWD + L_RCW * (C_W/C_R);

  L_WS[E_FrontRight] = pow((L_B * L_B + L_C * L_C), 0.5);
  L_WS[E_FrontLeft]  = pow((L_B * L_B + L_D * L_D), 0.5);
  L_WS[E_RearLeft]   = pow((L_A * L_A + L_D * L_D), 0.5);
  L_WS[E_RearRight]  = pow((L_A * L_A + L_C * L_C), 0.5);

  L_WA[E_FrontRight] = atan2(L_B, L_C) *180/C_PI;
  L_WA[E_FrontLeft]  = atan2(L_B, L_D) *180/C_PI;
  L_WA[E_RearLeft]   = atan2(L_A, L_D) *180/C_PI;
  L_WA[E_RearRight]  = atan2(L_A, L_C) *180/C_PI;

  L_Max = L_WS[E_FrontRight];

  if (L_WS[E_FrontLeft] > L_Max) {
    L_Max = L_WS[E_FrontLeft];
  }
 if (L_WS[E_RearLeft] > L_Max) {
   L_Max = L_WS[E_RearLeft];
  }
   if (L_WS[E_RearRight] > L_Max) {
     L_Max = L_WS[E_RearRight];
  }
  if (L_Max > 1) {
      L_WS[E_FrontRight] /= L_Max;
      L_WS[E_FrontLeft] /= L_Max;
      L_WS[E_RearLeft] /= L_Max;
      L_WS[E_RearRight] /= L_Max;
  }

  L_Gain = 0.1;
  if (Joystick_Gain > L_Gain)
    {
    L_Gain = Joystick_Gain;
    }

  L_WS[E_FrontRight] *= (K_WheelMaxSpeed * L_Gain);
  L_WS[E_FrontLeft]  *= (K_WheelMaxSpeed * L_Gain);
  L_WS[E_RearLeft]   *= (K_WheelMaxSpeed * L_Gain);
  L_WS[E_RearRight]  *= (K_WheelMaxSpeed * L_Gain);

  for (index = E_FrontLeft;
       index < E_RobotCornerSz;
       index = T_RobotCorner(int(index) + 1))
  {
    L_WA_FWD = DtrmnEncoderRelativeToCmnd(L_WA[index],
                                          V_WheelAngleFwd[index]);

    L_WA_FWD_Delta = fabs(L_WA[index] - L_WA_FWD);

    L_WA_REV = DtrmnEncoderRelativeToCmnd(L_WA[index],
                                          V_WheelAngleRev[index]);

    L_WA_REV_Delta = fabs(L_WA[index] - L_WA_REV);

    if (L_WA_FWD_Delta <= L_WA_REV_Delta)
      {
        V_WheelAngleArb[index] = L_WA_FWD;
      }
    else
      {
        V_WheelAngleArb[index] = L_WA_REV;
        L_WS[index] *= (-1); // Need to flip sign of drive wheel to acount for reverse direction
      }
  }
}
