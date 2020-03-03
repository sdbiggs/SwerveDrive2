#pragma once

#include <string>

#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>

class PIDConfig {
	public:
		double p,i,d;
		double pmax = 1,pmin = 1;
		double imax = 1,imin = 1;
		double dmax = 1,dmin = 1;
		double izone = 0;
    	double feedforward = 0;
    	double max = 1,min = -1;
	
		PIDConfig();
		PIDConfig(double P, double I, double D);
		
		/**
		 * Outputs PIDConfig values and allows them to be changed
		 *
		 * @param DebugTabName Give a name for the shuffleboard tab
		 */
		void Debug(std::string DebugTabName);
	private:
		bool init;
		nt::NetworkTableEntry nt_p, nt_i, nt_d;
		nt::NetworkTableEntry nt_pmax, nt_pmin;
		nt::NetworkTableEntry nt_imax, nt_imin;
		nt::NetworkTableEntry nt_dmax, nt_dmin;
		nt::NetworkTableEntry nt_izone;
		nt::NetworkTableEntry nt_ff;
		nt::NetworkTableEntry nt_max, nt_min;
		
};

PIDConfig::PIDConfig() {}

PIDConfig::PIDConfig(double P, double I, double D) {p = P;i = I;d = D;}

void PIDConfig::Debug(std::string DebugTabName)
{
	static auto& Tab = frc::Shuffleboard::GetTab(DebugTabName);
	
	if(!init)
	{
		nt_p = Tab.Add("P Gain", p).WithPosition(0,0).GetEntry();
		nt_i = Tab.Add("D Gain", d).WithPosition(1,0).GetEntry();
		nt_d = Tab.Add("I Gain", i).WithPosition(2,0).GetEntry();

		nt_pmax = Tab.Add("P Max", pmax).WithPosition(0,2).GetEntry();
		nt_pmin = Tab.Add("P Min", pmin).WithPosition(0,3).GetEntry();

		nt_imax = Tab.Add("I Max", imax).WithPosition(1,2).GetEntry();
		nt_imin = Tab.Add("I Min", imin).WithPosition(1,3).GetEntry();

		nt_dmax = Tab.Add("D Max", dmax).WithPosition(2,2).GetEntry();
		nt_dmin = Tab.Add("D Min", dmin).WithPosition(2,3).GetEntry();

		nt_izone = Tab.Add("I zone", izone).WithPosition(5,0).GetEntry();
		nt_ff = Tab.Add("Feed Forward", feedforward).WithPosition(6,0).GetEntry();

		nt_max = Tab.Add("Max", max).WithPosition(4,2).GetEntry();
		nt_min = Tab.Add("Min", min).WithPosition(4,3).GetEntry();

		init = true;
	} 
	
	p = nt_p.GetDouble(p);
	i = nt_i.GetDouble(i);
	d = nt_d.GetDouble(d);

	pmax = nt_pmax.GetDouble(pmax);
	pmin = nt_pmin.GetDouble(pmin);

	imax = nt_imax.GetDouble(imax);
	imin = nt_imin.GetDouble(imin);
	
	dmax = nt_dmax.GetDouble(dmax);
	dmin = nt_dmin.GetDouble(dmin);
	
	izone = nt_izone.GetDouble(izone);
	feedforward = nt_ff.GetDouble(feedforward);

	max = nt_max.GetDouble(max);
	min = nt_min.GetDouble(min);
	
}