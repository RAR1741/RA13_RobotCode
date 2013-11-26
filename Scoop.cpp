#include "Scoop.h"

Scoop::Scoop(int scoopMotor)
{
		_scoopMotor = new CANJaguar(scoopMotor);
}
	
void Scoop::ManualControl(double speed)
{
		_scoopMotor->Set(speed);
}
