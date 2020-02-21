#include "ColorSensor.hpp"
#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <string>

double desiredColor;
double spinNumberActual;
double spinNumberDesired = frc::SmartDashboard::GetNumber("Wheel Spin Number", 0);

// std::string gameData;
std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

void WheelOfFortune()
{
    if(c_joyStick2.GetRawButton(1) == true) 
    {
    if(detectedColor == kBlueTarget && spinNumberActual > spinNumberDesired) 
    {
      spinNumberActual = spinNumberActual + 1/2;
      Robot::m_fortuneWheel.Set(ControlMode::PercentOutput, 0.1);
    }
    else if(spinNumberActual == spinNumberDesired) 
     {
        Robot::m_fortuneWheel.Set(ControlMode::PercentOutput, 0);
     }    
}
else
{
     Robot::m_fortuneWheel.Set(ControlMode::PercentOutput, 0);
}        


if(c_joyStick2.GetRawButton(2) == true) {
    if (gameData.length() > 0) {
        
        switch (gameData[0]) {

            case 'R' :
                if (matchedColor == kBlueTarget) {
                    Robot::m_fortuneWheel.Set(ControlMode::PercentOutput, 0);
                }
                else {
                    Robot::m_fortuneWheel.Set(ControlMode::PercentOutput, 0.1);
                }
                break;
            
            case 'G' :
                if (matchedColor == kYellowTarget) {
                    Robot::m_fortuneWheel.Set(ControlMode::PercentOutput, 0);
                }
                else {
                    Robot::m_fortuneWheel.Set(ControlMode::PercentOutput, 0.1);
                }
                break;

            case 'B' :
                if (matchedColor == kRedTarget) {
                    Robot::m_fortuneWheel.Set(ControlMode::PercentOutput, 0);
                }
                else {
                    Robot::m_fortuneWheel.Set(ControlMode::PercentOutput, 0.1);
                }
                break;
            
            case 'Y' :
                if (matchedColor == kGreenTarget) {
                    Robot::m_fortuneWheel.Set(ControlMode::PercentOutput, 0);
                }
                else {
                    Robot::m_fortuneWheel.Set(ControlMode::PercentOutput, 0.1);
                }
                break;
            }
         }
    else{
            Robot::m_fortuneWheel.Set(ControlMode::PercentOutput, 0);
        }
}
}

