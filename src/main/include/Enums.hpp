/*
  Enums.hpp

   Created on: Jan 3, 2020
   Author: 5561
 */

#pragma once

#ifndef ENUMS
#define ENUMS

typedef enum T_RobotCorner
{
  E_FrontLeft,
  E_FrontRight,
  E_RearLeft,
  E_RearRight,
  E_RobotCornerSz
} T_RobotCorner;

typedef enum T_RoboShooter
{
  E_TopShooter,
  E_BottomShooter,
  E_RoboShooter
} T_RoboShooter;

// typedef enum T_WheelOfFortuneColor
// {
//   E_Red,
//   E_Yellow,
//   E_Blue,
//   E_Green,
//   E_Unknown
// } T_WheelOfFortuneColor;

typedef enum T_PID_Cal
{
  E_P_Gx,
  E_I_Gx,
  E_D_Gx,
  E_P_Ul,
  E_P_Ll,
  E_I_Ul,
  E_I_Ll,
  E_D_Ul,
  E_D_Ll,
  E_Max_Ul,
  E_Max_Ll,
  E_PID_CalSz
} T_PID_Cal;

typedef enum T_AutoTargetStates
{
  E_NotActive,
  E_TargetFoundRotateBotAndRollerSpinUp,
  E_MoveBallsToRollers,
  E_AutoTargetStatesSz
} T_AutoTargetStates;

#endif
