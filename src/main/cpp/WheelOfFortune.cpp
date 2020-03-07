#include "Robot.h"
// #include "ColorSensor.hpp"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "WheelOfFortune.hpp"
// #include "Enums.hpp"
// #include "Const.hpp"
// #include <string>

double desiredColor;
double spinNumberActual;
// double spinNumberDesired = frc::SmartDashboard::GetNumber("Wheel Spin Number", 0);
// double spinNumberDesired;
// m_fortuneWheel.Set(ControlMode::PercentOutput, 0.1);

// std::string gameData;
// std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
std::string gameData;

//make sure L_Blue etc is actually connected to a color


/******************************************************************************
 * Function:     WheelOfFortune
 *
 * Description:  This function controls the control panel, aka Wheel of Fortune.
 ******************************************************************************/
double WheelOfFortune(//T_WheelOfFortuneColor L_WheelOfFortuneColor,
                      bool                  L_AutoControl,
                      bool                  L_ManualFwd,
                      bool                  L_ManualRev)
  {
  double L_FortuneWheelPower = 0;


  
  gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  frc::SmartDashboard::PutString("Needed Color!", gameData);


if (L_ManualFwd == true && L_AutoControl == false && L_ManualRev == false) {
    L_FortuneWheelPower = K_WheelOfFortunePwr;
}
else if (L_ManualRev == true && L_ManualFwd == false && L_AutoControl == false) {
    L_FortuneWheelPower = -K_WheelOfFortunePwr;
}
else if (L_ManualFwd == false && L_AutoControl == false && L_ManualRev == false) {
    L_FortuneWheelPower = 0.0;
}




// if (L_AutoControl == true) {
//     if (L_WheelOfFortuneColor == E_Blue && spinNumberActual > L_SpinNumberDesired)
//       {
//       spinNumberActual = spinNumberActual + 1/2;
//       L_FortuneWheelPower = 0.1;
//       }
//     else if(spinNumberActual == L_SpinNumberDesired)
//       {
//       L_FortuneWheelPower = 0.0;
//       }
//     else {
//    L_FortuneWheelPower = K_WheelOfFortunePwr;
//     }
// }
// else
// {
// L_FortuneWheelPower = 0.0;
// }




  // if(L_AutoControl == true)
  //   {
  //   if (gameData.length() > 0)
  //     {
  //     switch (gameData[0])
  //       {
  //       case 'R' :
  //         if (L_WheelOfFortuneColor == E_Blue)
  //           {
  //           L_FortuneWheelPower = 0.0;
  //           }
  //         else
  //           {
  //           L_FortuneWheelPower = K_WheelOfFortunePwr;
  //           }
  //         break;

  //       case 'G' :
  //         if (L_WheelOfFortuneColor == E_Yellow)
  //           {
  //           L_FortuneWheelPower = 0.0;
  //           }
  //         else
  //           {
  //           L_FortuneWheelPower = K_WheelOfFortunePwr;
  //           }
  //         break;

  //       case 'B' :
  //         if (L_WheelOfFortuneColor == E_Red)
  //           {
  //           L_FortuneWheelPower = 0.0;
  //           }
  //         else
  //           {
  //           L_FortuneWheelPower = K_WheelOfFortunePwr;
  //           }
  //         break;

  //       case 'Y' :
  //         if (L_WheelOfFortuneColor == E_Green)
  //           {
  //           L_FortuneWheelPower = 0.0;
  //           }
  //         else
  //           {
  //           L_FortuneWheelPower = K_WheelOfFortunePwr;
  //           }
  //         break;
  //       }
  //     }
  //   else
  //     {
  //     L_FortuneWheelPower = 0.0;
  //     }
  //   }
    return (L_FortuneWheelPower);
  }