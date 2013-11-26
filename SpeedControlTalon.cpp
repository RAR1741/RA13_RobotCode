#include "SpeedControlTalon.h"
#include "WPILib.h"

#include <iostream>
using namespace std;
//methods
SpeedControlTalon::SpeedControlTalon(int talPWMChannel, int encoderA, int encoderB)
{
	tal = new Talon(talPWMChannel);
	encoder = new Encoder(encoderA,encoderB,false,Encoder::k4X);
	encoder->SetPIDSourceParameter(Encoder::kRate);
	encoder->SetDistancePerPulse((double)60.0/LINESPERREV); //Converts encoder GetRate() to RPM
	pid = new PIDController(0.0,0.0,0.0,encoder,tal,0.01);
	pid->SetInputRange(-m_MAXSPEED,m_MAXSPEED);
	pid->SetOutputRange(-1.0,1.0);
	encoder->Start();
}
void SpeedControlTalon::SetPID(double p, double i, double d)
{
	pid->SetPID(p,i,d);
}
void SpeedControlTalon::Set(float target)
{
	pid->SetSetpoint(target);
}

void SpeedControlTalon::EnablePID()
{
	pid->Enable();
	encoder->Start();
	
}
void SpeedControlTalon::DisablePID()
{
	pid->Disable();
	encoder->Stop();	
}
void SpeedControlTalon::ResetPID()
{
	pid->Reset();
}

//accessors
double SpeedControlTalon::GetP()
{
	return pid->GetP();
}
double SpeedControlTalon::GetI()
{
	return pid->GetI();
}
double SpeedControlTalon::GetD()
{
	return pid->GetD();
}
