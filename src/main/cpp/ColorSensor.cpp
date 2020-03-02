// #include "rev/ColorSensorV3.h" //
// #include "rev/ColorMatch.h" //
// #include "frc/smartdashboard/SmartDashboard.h"
// #include "ColorSensor.hpp"
// #include "Enums.hpp"

// static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
// std::string colorString;
// rev::ColorSensorV3 m_colorSensor{i2cPort};
// rev::ColorMatch m_colorMatcher;
// // static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429); //
// // static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240); //
// // static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114); //
// // static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113); //

// T_WheelOfFortuneColor ColorSensor(bool init)
//   {
//     T_WheelOfFortuneColor L_WheelOfFortuneColor;

//     m_colorMatcher.AddColorMatch(kBlueTarget);
//     m_colorMatcher.AddColorMatch(kGreenTarget);
//     m_colorMatcher.AddColorMatch(kRedTarget);
//     m_colorMatcher.AddColorMatch(kYellowTarget);

//     double IR = m_colorSensor.GetIR();
//     uint32_t proximity = m_colorSensor.GetProximity();
//     double proximity_1 = (double)proximity;

//     frc::Color detectedColor = m_colorSensor.GetColor();

//     /**
//      * Run the color match algorithm on our detected color
//      */
//     double confidence = 0.0;
//     frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
//     if (proximity_1 < 450) {
//         if (matchedColor == kBlueTarget) {
//                 colorString = "Blue";
//                 L_WheelOfFortuneColor = E_Blue;
//             } else if (matchedColor == kRedTarget) {
//                 colorString = "Red";
//                 L_WheelOfFortuneColor = E_Red;
//             } else if (matchedColor == kGreenTarget) {
//                 colorString = "Green";
//                 L_WheelOfFortuneColor = E_Green;
//             } else if (matchedColor == kYellowTarget) {
//                 colorString = "Yellow";
//                 L_WheelOfFortuneColor = E_Yellow;
//             } else {
//                 colorString = "Unknown";
//                 L_WheelOfFortuneColor = E_Unknown;
//             } 
//         } else {
//             colorString = "Too far away";
//             L_WheelOfFortuneColor = E_Unknown;
//         } 

//     frc::SmartDashboard::PutNumber("Proximity", proximity_1);
//     frc::SmartDashboard::PutNumber("IR", IR);
//     frc::SmartDashboard::PutNumber("Red", detectedColor.red);
//     frc::SmartDashboard::PutNumber("Green", detectedColor.green);
//     frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
//     frc::SmartDashboard::PutNumber("Confidence", confidence);
//     frc::SmartDashboard::PutString("Detected Color", colorString);

//     return (L_WheelOfFortuneColor);
// }