#pragma once

#include <string>

#include "PIDConfig.hpp"

#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>

class ShooterConfig
{
    public:
        PIDConfig topShooterPIDConfig;
        PIDConfig bottomShooterPIDConfig;
        double TopDesiredSpeed;
        double BottomDesiredSpeed;

        ShooterConfig();

        ShooterConfig(PIDConfig TopShooterPIDConfig, PIDConfig BottomShooterPIDConfig);

        void Debug(std::string TabName);
    private:
        bool init;
        nt::NetworkTableEntry nt_topDesiredSpeed, nt_bottomDesiredSpeed;
};

ShooterConfig::ShooterConfig(){}

ShooterConfig::ShooterConfig(PIDConfig TopShooterPIDConfig, PIDConfig BottomShooterPIDConfig)
{
    topShooterPIDConfig = TopShooterPIDConfig;
    bottomShooterPIDConfig = BottomShooterPIDConfig;
}

void ShooterConfig::Debug(std::string TabName)
{
    static auto& Tab = frc::Shuffleboard::GetTab(TabName);

    topShooterPIDConfig.Debug("(Top PID)"+TabName);
    bottomShooterPIDConfig.Debug("(Bottom PID)"+TabName);

    if(!init)
    {
        nt_topDesiredSpeed = Tab.Add("Top Desired Speed", TopDesiredSpeed).WithPosition(0,0).GetEntry();
        nt_bottomDesiredSpeed = Tab.Add("Bottom Desired Speed", BottomDesiredSpeed).WithPosition(0,1).GetEntry();

        init = true;
    }

    TopDesiredSpeed = nt_topDesiredSpeed.GetDouble(TopDesiredSpeed);
    BottomDesiredSpeed = nt_bottomDesiredSpeed.GetDouble(BottomDesiredSpeed);

}
