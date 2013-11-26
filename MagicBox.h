#include "WPILib.h"

class MagicBox
{
public:
	MagicBox(int gyroChannel);
	
	double getGyroAngle();
	void resetGyro();
private:
	Gyro * gyro;
};
