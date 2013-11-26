#include "MagicBox.h"

MagicBox::MagicBox(int gyroChannel)
{
	gyro = new Gyro(gyroChannel);
}

double MagicBox::getGyroAngle()
{
	return gyro->GetAngle() + 180.0;
}

void MagicBox::resetGyro()
{
	gyro->Reset();
}
