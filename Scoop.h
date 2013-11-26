#ifndef SCOOP_H
#define SCOOP_H

#include <wpilib.h>

class Scoop
{
	public:
	Scoop(int scoopMotor);
	void ManualControl(double speed);
	
	private:
	SpeedController* _scoopMotor;
};
#endif
