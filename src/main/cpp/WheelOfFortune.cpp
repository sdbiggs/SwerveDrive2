#include "WheelOfFortune.hpp"
#include "ColorSensor.hpp"
#include "Robot.h"


enum colorModePick {rotation, colorPick};
double primaryColor = (matchedColor); //need to have a way to make it not switch primary color
double counter;
double desiredColor;

desiredColor = frc::SmartDashboard::GetNumber("Desired Color", 0);
std::string gameData;
gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();


    if(c_joyStick2.GetRawButton(1) == true)
        {
        m_intake.Set(ControlMode::PercentOutput, 0.2);
        }
     else {
        m_intake.Set(ControlMode::PercentOutput, 0);
        }

       if (detectedColor == primaryColor) {
          for (detectedColor == primaryColor) {
              counter++;
            }
        }
    

    if (gameData.length() > 0) {
       
            switch (gameData[0]) {

            case 'R' :
                if (matchedColor == kRedTarget) {
                    //a
                }
                break;
            
            case 'G' :
                if (matchedColor == kGreenTarget) {
                    //a
                }
                break;

            case 'B' :
                if (matchedColor == kBlueTarget) {
                   // a
                }
                break;
            
            case 'Y' :
                if (matchedColor == kYellowTarget) {
                    //a
                }
                break;

        else {
             m_intake.Set(ControlMode::PercentOutput, 0);
            }
        }
    }
    