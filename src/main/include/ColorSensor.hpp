#include <string>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include "Enums.hpp"

// static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
// std::string colorString;
// rev::ColorSensorV3 m_colorSensor{i2cPort};
// rev::ColorMatch m_colorMatcher;

// T_WheelOfFortuneColor ColorSensor(bool init);

// void ColorSensor (bool init);
// extern std::string colorString;

static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);