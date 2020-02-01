/*
  LookUp.hpp

   Created on: Feb 14, 2018
       Author: sdbig
 */
#ifndef SRC_ROBORIO2018_LOOKUP_HPP_
#define SRC_ROBORIO2018_LOOKUP_HPP_

extern double LookUp1D_Table(const double *L_X_Axis,
                             const double *L_TableData1D,
                                   int     L_AxisSize,
                                   int     L_CalArraySize,
                                   double  L_Input);

extern double DesiredSpeed(double L_JoystickAxis);

#endif /* SRC_ROBORIO2018_LOOKUP_HPP_ */
