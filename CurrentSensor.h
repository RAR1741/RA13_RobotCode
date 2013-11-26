/**
 * CurrentSensor.h
 * 
 * Senses... current?
 */

#ifndef CURRENT_SENSOR_H__
#define CURRENT_SENSOR_H__

#include <wpilib.h>

class CurrentSensor : public AnalogChannel 
{
public:
	CurrentSensor(int ch);
	CurrentSensor(int mod, int ch);
	virtual ~CurrentSensor();
	
	double GetCurrent();
	
private:
	static const double kAmpsPerVolt;
};

#endif
