#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "AHRS.h"
#include "Const.hpp"

AHRS *NavX;

using namespace frc;

double gyro_angleprev;
double gryo_loopcount = 0;
double gyro_yawangledegrees;
double gyro_yawanglerad;
double gyro_rolloverrad;

void GyroRobotInit()
{
    try{
      NavX = new AHRS(SPI::Port::kMXP);
    }
    catch(const std::exception e){
      std::string err_string = "Error instantiating navX-MXP:  ";
      err_string += e.what();
      DriverStation::ReportError(err_string.c_str());
    }
}

void GyroZero()
{
    NavX->ZeroYaw();
}

void Gyro() {

  double gyro_currentyaw = (double)NavX->GetYaw();
  //Check to see if gyro angle flips over 180 or -180
  if(175 <= abs(gyro_angleprev))
  {
    if(gyro_angleprev < 0 && gyro_currentyaw > 0)
    {
      gryo_loopcount -= 1;
    } else if (gyro_angleprev > 0 && gyro_currentyaw < 0)
    {
      gryo_loopcount += 1;
    }
  }
  gyro_rolloverrad = ((gryo_loopcount * 360) + gyro_currentyaw) / C_RadtoDeg;

  gyro_yawangledegrees = (double)NavX->GetYaw();
  gyro_yawanglerad = (double)NavX->GetYaw() / C_RadtoDeg;


  gyro_angleprev = gyro_currentyaw;

}
