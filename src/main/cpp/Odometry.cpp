/*
 * Odometry.cpp
 * 
 * Team 5561 2020 Code
 *
 * This code is meant track the field position of the robot based on the gyro and encoders:
 * - Started on 02/22/2021.  Built, untested.
 *
 * */

#include "vision.hpp"
#include "Gyro.hpp"
#include "Robot.h"
#include "Lookup.hpp"
#include <iostream>

using namespace frc;


void DtrmnSwerveBotLocation(bool    L_b_RobotInInit,
                            double  L_Deg_Gyro,
                            double *L_Deg_WheelAngleArb,
                            double *L_M_DeltaWheelDistance,
                            double *L_M_RobotDisplacementX,
                            double *L_M_RobotDisplacementY)
  {
    T_RobotCorner L_e_Index;
    double        L_Deg_RelativeAngle[E_RobotCornerSz];
    double        L_M_DeltaCornerDisplacementX[E_RobotCornerSz];
    double        L_M_DeltaCornerDisplacementY[E_RobotCornerSz];
    double        L_M_TotalDeltaX = 0;
    double        L_M_TotalDeltaY = 0;

    if (L_b_RobotInInit == false)
      {
        for (L_e_Index = E_FrontLeft;
             L_e_Index < E_RobotCornerSz;
             L_e_Index = T_RobotCorner(int(L_e_Index) + 1))
          {
              L_Deg_RelativeAngle[L_e_Index] = L_Deg_Gyro + L_Deg_WheelAngleArb[L_e_Index];
              L_M_DeltaCornerDisplacementX[L_e_Index] = sin(L_Deg_RelativeAngle[L_e_Index]) * L_M_DeltaWheelDistance[L_e_Index];
              L_M_DeltaCornerDisplacementY[L_e_Index] = cos(L_Deg_RelativeAngle[L_e_Index]) * L_M_DeltaWheelDistance[L_e_Index];

              L_M_TotalDeltaX += L_M_DeltaCornerDisplacementX[L_e_Index];
              L_M_TotalDeltaY += L_M_DeltaCornerDisplacementY[L_e_Index];
          }
    
        L_M_TotalDeltaX = L_M_TotalDeltaX / 4;
        L_M_TotalDeltaY = L_M_TotalDeltaY / 4;

        *L_M_RobotDisplacementX += L_M_TotalDeltaX;
        *L_M_RobotDisplacementY += L_M_TotalDeltaY;
      }
  }


