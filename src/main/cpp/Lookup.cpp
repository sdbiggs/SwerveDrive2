#include "Const.hpp"

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

  double DesiredSpeed(double L_JoystickAxis)
  {
  double L_DesiredDriveSpeed = 0.0;
  int L_AxisSize = (int)(sizeof(K_DesiredDriveSpeedAxis) / sizeof(K_DesiredDriveSpeed[0]));
  int L_CalArraySize = (int)(sizeof(K_DesiredDriveSpeed) / sizeof(K_DesiredDriveSpeed[0]));

  L_DesiredDriveSpeed = LookUp1D_Table(&K_DesiredDriveSpeedAxis[0],
                                       &K_DesiredDriveSpeed[0],
                                       L_AxisSize,
                                       L_CalArraySize,
                                       L_JoystickAxis);

  return L_DesiredDriveSpeed;
  }
