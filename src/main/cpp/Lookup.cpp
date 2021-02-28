/*
  Lookup.cpp

  Created on: Jan 3, 2020
  Author: 5561

  This file contains functions related to lookup and interpolation.

 */
#include "Const.hpp"
#include <math.h>

/******************************************************************************
 * Function:     LookUp1D_Table
 *
 * Description:  Single dimension lookup table.
 ******************************************************************************/
double LookUp1D_Table(const double *L_X_Axis,
                      const double *L_TableData1D,
                            int     L_AxisSize,
                            int     L_CalArraySize,
                            double  L_Input)
  {
  int    L_Index        = 0;
  double L_LookupX1     = 0.0;
  double L_LookupX2     = 0.0;
  double L_LookupX_Diff = 0.0;
  double L_LookupY1     = 0.0;
  double L_LookupY2     = 0.0;
  double L_LookupY_Diff = 0.0;
  double L_LookupDiv    = 0.0;
  bool L_LookupPt1Found = false;
  double L_Output       = 0.0;

  /* Table length MUST equal axis length. */
  if (L_CalArraySize == L_AxisSize)
    {
    if (L_Input >= (L_X_Axis[L_AxisSize - 1]))
      {
      // We have gone off or are at the end of the axis
      return (L_TableData1D[L_AxisSize - 1]);
      }
    else if (L_Input <= (L_X_Axis[0]))
      {
      // We have gone off or are at the beginning of the axis
      return (L_TableData1D[0]);
      }
    else
      {
      for (L_Index = 0; ((L_Index < (L_AxisSize - 1)) && (L_LookupPt1Found == false)) ; L_Index++)
        {
        if ((L_Input >= L_X_Axis[L_Index])     &&
            (L_Input <  L_X_Axis[L_Index + 1]) &&
            (L_LookupPt1Found == false))
          {
          L_LookupX1 = L_X_Axis[L_Index];
          L_LookupY1 = L_TableData1D[L_Index];
          L_LookupX2 = L_X_Axis[L_Index + 1];
          L_LookupY2 = L_TableData1D[L_Index + 1];
          L_LookupPt1Found = true;

          L_Index = L_AxisSize;
          }
        }

      if ((L_LookupPt1Found == true))
        {
        L_LookupX_Diff = L_LookupX2 - L_LookupX1;
        L_LookupY_Diff = L_LookupY2 - L_LookupY1;
        if (L_LookupX_Diff != 0.0)
          {
          /* Protect for zero division */
          L_LookupDiv = L_LookupY_Diff / L_LookupX_Diff;
          }
        else
          {
          L_LookupDiv = 0.0;
          }
        L_Output = L_LookupY1 + (L_Input-L_LookupX1) * L_LookupDiv;

        return L_Output;
        }
      }
    }

  // Not in range...
  return 0;
  }


/******************************************************************************
 * Function:     RampTo
 *
 * Description:  Function to determine the roller speed, aka the "special
 *               beam cannon".  This is a function of distance out from the
 *               target and the angle of the robot relative to the target.
 ******************************************************************************/
double RampTo(double  L_Final,
              double  L_Current,
              double  L_Slope)
  {
  if (L_Final - L_Current > 0)
    {
    L_Current += L_Slope;

    if (L_Current >= L_Final)
      {
      L_Current = L_Final;
      }
    }
  else if (L_Final - L_Current < 0)
    {
    L_Current -= L_Slope;
    if (L_Current <= L_Final)
      {
      L_Current = L_Final;
      }
    }
  else
    {
    L_Current = L_Final;
    }
  return (L_Current);
  }


/******************************************************************************
 * Function:     CriteriaMet
 *
 * Description:  This function checks to see if certain criteria is met.
 ******************************************************************************/
bool CriteriaMet(double  L_Desired,
                 double  L_Current,
                 double  L_AllowedError)
  {
  bool L_CriteriaMet = true;

  if (fabs(L_Current - L_Desired) <= L_AllowedError)
    {
    L_CriteriaMet = false;
    }

  return (L_CriteriaMet);
  }


/******************************************************************************
 * Function:     LookUp1D_Axis
 *
 * Description:  Single axis lookup.
 ******************************************************************************/
void LookUp1D_Axis(const double *L_Axis,
                         int     L_AxisSize,
                         int    *L_Index_i,
                         int    *L_Index_j,
                         double  L_Input,
                         double *L_InputScalar,
                         double *L_InputScalar1Minus)
  {
  int    L_Index          = 0;
  bool   L_LookupPt1Found = false;
  double L_Denomenator    = 0.0;

  if (L_Input >= (L_Axis[L_AxisSize - 1]))
    {
    // We have gone off or are at the end of the axis
    *L_Index_i     = L_AxisSize - 2;
    *L_Index_j     = L_AxisSize - 1;
    *L_InputScalar = 1;
    *L_InputScalar1Minus = 0;
    }
  else if (L_Input <= (L_Axis[0]))
    {
    // We have gone off or are at the beginning of the axis
    *L_Index_i     = 0;
    *L_Index_j     = 1;
    *L_InputScalar = 0;
    *L_InputScalar1Minus = 1;
    }
  else
    {
    for (L_Index = 0; ((L_Index < (L_AxisSize - 1)) && (L_LookupPt1Found == false)) ; L_Index++)
      {
      if ((L_Input >= L_Axis[L_Index])     &&
          (L_Input <  L_Axis[L_Index + 1]) &&
          (L_LookupPt1Found == false))
        {
        L_LookupPt1Found = true;
        *L_Index_i        = L_Index;
        *L_Index_j        = L_Index + 1;
        L_Denomenator = L_Axis[L_Index + 1] - L_Axis[L_Index];
        if (L_Denomenator != 0.0)
          {
          /* Protect for zero division */
          *L_InputScalar1Minus = (L_Axis[L_Index + 1] - L_Input) / (L_Denomenator);
          }
        else
          {
          *L_InputScalar1Minus = 1;
          }
        *L_InputScalar = 1 - *L_InputScalar1Minus;
        }
      }

    if (L_LookupPt1Found == false)
      {
      /* Defensive programming.  We really shouldn't reach here... */
      *L_Index_i           = 0;
      *L_Index_j           = 1;
      *L_InputScalar       = 0;
      *L_InputScalar1Minus = 1 - *L_InputScalar;
      }
    }
  }


/******************************************************************************
 * Function:     LookUp2D_Table
 *
 * Description:  Two dimension lookup table.  Based on the linear interpolation
 *               at the link below:
 *               https://en.wikipedia.org/wiki/Bilinear_interpolation
 ******************************************************************************/
double LookUp2D_Table(double const *L_X_Axis,
                      int           L_X_AxisSize,
                      double        L_X_Input,
                      double const *L_Y_Axis,
                      int           L_Y_AxisSize,
                      double        L_Y_Input,
                      double      **L_TableData2D)
  {
  int    L_X_Index_i           = 0;
  int    L_X_Index_j           = 0;
  double L_X_IndexScalar       = 0.0;
  double L_X_IndexScalar1Minus = 0.0;
  int    L_Y_Index_i           = 0;
  int    L_Y_Index_j           = 0;
  double L_Y_IndexScalar       = 0.0;
  double L_Y_IndexScalar1Minus = 0.0;
  double L_F_XiYi              = 0.0;
  double L_F_XiYj              = 0.0;
  double L_F_XjYi              = 0.0;
  double L_F_XjYj              = 0.0;
  double L_Output              = 0.0;

  LookUp1D_Axis(&L_X_Axis[0],
                 L_X_AxisSize,
                &L_X_Index_i,
                &L_X_Index_j,
                 L_X_Input,
                &L_X_IndexScalar,
                &L_X_IndexScalar1Minus);

  LookUp1D_Axis(&L_Y_Axis[0],
                 L_Y_AxisSize,
                &L_Y_Index_i,
                &L_Y_Index_j,
                 L_Y_Input,
                &L_Y_IndexScalar,
                &L_Y_IndexScalar1Minus);

  L_F_XiYi = L_TableData2D[(int)L_X_Index_i][(int)L_Y_Index_i];
  L_F_XiYj = L_TableData2D[(int)L_X_Index_i][(int)L_Y_Index_j];
  L_F_XjYi = L_TableData2D[(int)L_X_Index_j][(int)L_Y_Index_i];
  L_F_XjYj = L_TableData2D[(int)L_X_Index_j][(int)L_Y_Index_j];

  L_Output = L_F_XiYi * L_X_IndexScalar1Minus * L_Y_IndexScalar1Minus +
             L_F_XjYi * L_X_IndexScalar       * L_Y_IndexScalar1Minus +
             L_F_XiYj * L_X_IndexScalar1Minus * L_Y_IndexScalar +
             L_F_XjYj * L_X_IndexScalar       * L_Y_IndexScalar;

  return (L_Output);
  }


/******************************************************************************
 * Function:     DesiredSpeed
 *
 * Description:  Function to scale the joystick input.
 *               Primarily used for smooth debouncing.
 ******************************************************************************/
double DesiredSpeed(double L_JoystickAxis)
  {
  double L_DesiredDriveSpeed = 0.0;
  int L_AxisSize             = (int)(sizeof(K_DesiredDriveSpeedAxis) / sizeof(K_DesiredDriveSpeed[0]));
  int L_CalArraySize         = (int)(sizeof(K_DesiredDriveSpeed) / sizeof(K_DesiredDriveSpeed[0]));

  L_DesiredDriveSpeed = LookUp1D_Table(&K_DesiredDriveSpeedAxis[0],
                                       &K_DesiredDriveSpeed[0],
                                        L_AxisSize,
                                        L_CalArraySize,
                                        L_JoystickAxis);

  return L_DesiredDriveSpeed;
  }

/******************************************************************************
 * Function:     DesiredUpperBeamSpeed
 *
 * Description:  Function to power up upper beam.
 ******************************************************************************/
double DesiredUpperBeamSpeed(double L_TargetDistance)
  {
  double L_DesiredBeamSpeed = 0.0;
  int L_AxisSize             = (int)(sizeof(K_DesiredDistanceAxis) / sizeof(K_DesiredSpeedUpperBeam[0]));
  int L_CalArraySize         = (int)(sizeof(K_DesiredSpeedUpperBeam) / sizeof(K_DesiredSpeedUpperBeam[0]));

  L_DesiredBeamSpeed = LookUp1D_Table(&K_DesiredDistanceAxis[0],
                                       &K_DesiredSpeedUpperBeam[0],
                                        L_AxisSize,
                                        L_CalArraySize,
                                        L_TargetDistance);

  return L_DesiredBeamSpeed;
  }

  /******************************************************************************
 * Function:     DesiredLowerBeamSpeed
 *
 * Description:  Function to power up lower beam.
 ******************************************************************************/
double DesiredLowerBeamSpeed(double L_TargetDistance)
  {
  double L_DesiredBeamSpeed = 0.0;
  int L_AxisSize             = (int)(sizeof(K_DesiredDistanceAxis) / sizeof(K_DesiredSpeedLowerBeam[0]));
  int L_CalArraySize         = (int)(sizeof(K_DesiredSpeedLowerBeam) / sizeof(K_DesiredSpeedLowerBeam[0]));

  L_DesiredBeamSpeed = LookUp1D_Table(&K_DesiredDistanceAxis[0],
                                       &K_DesiredSpeedLowerBeam[0],
                                        L_AxisSize,
                                        L_CalArraySize,
                                        L_TargetDistance);

  return L_DesiredBeamSpeed;
  }

/******************************************************************************
 * Function:     DesiredRotateSpeed
 *
 * Description:  Function to determine the speed at which to rotate the robot 
 *               (for auto targeting and auto rotate).
 ******************************************************************************/
double DesiredRotateSpeed(double L_Error)
  {
  double L_DesiredRotateSpeed = 0.0;
  int L_AxisSize             = (int)(sizeof(K_DesiredRotateSpeedAxis) / sizeof(K_DesiredRotateSpeed[0]));
  int L_CalArraySize         = (int)(sizeof(K_DesiredRotateSpeed) / sizeof(K_DesiredRotateSpeed[0]));

  L_DesiredRotateSpeed = LookUp1D_Table(&K_DesiredRotateSpeedAxis[0],
                                        &K_DesiredRotateSpeed[0],
                                         L_AxisSize,
                                         L_CalArraySize,
                                         L_Error);

  return L_DesiredRotateSpeed;
  }

/******************************************************************************
 * Function:     DesiredAutoRotateSpeed
 *
 * Description:  Function to determine the speed at which to rotate the robot 
 *               (for auto targeting and auto rotate).
 ******************************************************************************/
double DesiredAutoRotateSpeed(double L_Error)
  {
  double L_DesiredRotateSpeed = 0.0;
  int L_AxisSize             = (int)(sizeof(K_DesiredAutoRotateSpeedAxis) / sizeof(K_DesiredAutoRotateSpeed[0]));
  int L_CalArraySize         = (int)(sizeof(K_DesiredAutoRotateSpeed) / sizeof(K_DesiredAutoRotateSpeed[0]));

  L_DesiredRotateSpeed = LookUp1D_Table(&K_DesiredAutoRotateSpeedAxis[0],
                                        &K_DesiredAutoRotateSpeed[0],
                                         L_AxisSize,
                                         L_CalArraySize,
                                         L_Error);

  return L_DesiredRotateSpeed;
  }

/******************************************************************************
 * Function:     DesiredRollerSpeed
 *
 * Description:  Function to determine the roller speed, aka the "special
 *               beam cannon".  This is a function of distance out from the
 *               target and the angle of the robot relative to the target.
 *               We also look up the ideal robot angle for targeting.
 ******************************************************************************/
void DesiredRollerSpeed(double  L_Distance,
                        double  L_Angle,
                        double *L_UpperCmnd,
                        double *L_LowerCmnd)
  {
  double  L_DesiredRobotAngle       = 0.0;
  double  L_DesiredRollerSpeedUpper = 0.0;
  double  L_DesiredRollerSpeedLower = 0.0;
  double *L_RollerSpeedCalibration[K_BallLauncherDistanceSz];
  double  L_Temp[K_BallLauncherDistanceSz][K_BallLauncherAngleSz];
  int     i;
  int     j;

  for (i = 0; i < K_BallLauncherDistanceSz; i++)
    {
    for (j = 0; j < K_BallLauncherAngleSz; j++)
      {
      L_Temp[i][j] = K_BallLauncherUpperSpeed[i][j];
      }
    L_RollerSpeedCalibration[i] = L_Temp[i];
    }

  L_DesiredRollerSpeedUpper = LookUp2D_Table(&K_BallLauncherDistanceAxis[0],
                                              K_BallLauncherDistanceSz,
                                              L_Distance,
                                             &K_BallLauncherAngleAxis[0],
                                              K_BallLauncherAngleSz,
                                              L_Angle,
                                              L_RollerSpeedCalibration);

  for (i = 0; i < K_BallLauncherDistanceSz; i++)
    {
    for (j = 0; j < K_BallLauncherAngleSz; j++)
      {
      L_Temp[i][j] = K_BallLauncherLowerSpeed[i][j];
      }
    L_RollerSpeedCalibration[i] = L_Temp[i];
    }

  L_DesiredRollerSpeedLower = LookUp2D_Table(&K_BallLauncherDistanceAxis[0],
                                              K_BallLauncherDistanceSz,
                                              L_Distance,
                                             &K_BallLauncherAngleAxis[0],
                                              K_BallLauncherAngleSz,
                                              L_Angle,
                                              L_RollerSpeedCalibration);

  *L_UpperCmnd  = L_DesiredRollerSpeedUpper;
  *L_LowerCmnd  = L_DesiredRollerSpeedLower;
  }
