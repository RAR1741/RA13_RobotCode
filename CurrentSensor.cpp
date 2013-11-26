#include "CurrentSensor.h"

// 2 mV = 1A
// 10 mV = 1A
// 1 A = .010 V
// 100 A = 1 V
const double CurrentSensor::kAmpsPerVolt = 100.0;
//#define kAmpsPerVolt (100.0)

CurrentSensor::CurrentSensor(int ch)
	: AnalogChannel(ch)
{
	
}

CurrentSensor::CurrentSensor(int mod, int ch)
	: AnalogChannel(mod, ch)
{
	
}

CurrentSensor::~CurrentSensor() {
	// 
}

double CurrentSensor::GetCurrent() {
	return this->GetVoltage() * kAmpsPerVolt;
}

