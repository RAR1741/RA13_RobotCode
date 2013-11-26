#include "SpeedControlVictor.h"
#include "WPILib.h"

#include <iostream>
using namespace std;
//methods
SpeedControlVictor::SpeedControlVictor(int vicPWMChannel, int encoderA, int encoderB)
{
	vic = new Victor(vicPWMChannel);
	encoder = new Encoder(encoderA,encoderB,false,Encoder::k4X);
	encoder->SetPIDSourceParameter(Encoder::kRate);
	encoder->SetDistancePerPulse((double)60.0/LINESPERREV); //Converts encoder GetRate() to RPM
	pid = new PIDController(0.0,0.0,0.0,encoder,vic,0.01);
	pid->SetInputRange(-m_MAXSPEED,m_MAXSPEED);
	pid->SetOutputRange(-1.0,1.0);
	encoder->Start();
}
void SpeedControlVictor::SetPID(double p, double i, double d)
{
	pid->SetPID(p,i,d);
}
void SpeedControlVictor::Set(float target)
{
	pid->SetSetpoint(target);
}

void SpeedControlVictor::EnablePID()
{
	pid->Enable();
	encoder->Start();
	
}
void SpeedControlVictor::DisablePID()
{
	pid->Disable();
	encoder->Stop();	
}
void SpeedControlVictor::ResetPID()
{
	pid->Reset();
}

//accessors
double SpeedControlVictor::GetP()
{
	return pid->GetP();
}
double SpeedControlVictor::GetI()
{
	return pid->GetI();
}
double SpeedControlVictor::GetD()
{
	return pid->GetD();
}
