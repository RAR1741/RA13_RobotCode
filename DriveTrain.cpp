#include "DriveTrain.h"
#include "CurrentSensor.h"
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>

using namespace std;


DriveTrain::DriveTrain(int fl, int fr, int rl, int rr)
{
	FLMotor = new SpeedControlTalon(fl, 5,6);
	FRMotor = new SpeedControlTalon(fr, 7,8);
	RLMotor = new SpeedControlTalon(rl, 9,10);
	RRMotor = new SpeedControlTalon(rr, 11,12);
	
	int offset = 2;
	
	FLSensor = new CurrentSensor(fl+offset);
	FRSensor = new CurrentSensor(fr+offset);
	RLSensor = new CurrentSensor(rl+offset);
	RRSensor = new CurrentSensor(rr+offset);
	
//	bool gyroDrive = true;

	SpeedControlTalon * motors[4] = { FLMotor, FRMotor, RLMotor, RRMotor };
	for (int i = 0; i < 4; ++i) {
		motors[i]->SetPID(DRIVE_KP,DRIVE_KI,DRIVE_KD); //Sets the proportional, integral, and derivative values for closed loop speed
	}
}

DriveTrain::~DriveTrain()
{
	
}

void DriveTrain::RotateVector(double &x, double &y, double angle)
{
	double cosA = cos(angle * (3.14159 / 180.0));
	double sinA = sin(angle * (3.14159 / 180.0));
	double xOut = x * cosA - y * sinA;
	double yOut = x * sinA + y * cosA;
	x = xOut;
	y = yOut;
}
void DriveTrain::setGyroDrive(bool b)
{
	gyroDrive = b;
}
void DriveTrain::Drive(double x, double y, double r, double gyro, bool speedModify,bool halfSpeed)
{
	if(!gyroDrive)
		gyro = 0;
	this->EnablePID();
	double xIn = DeadZone(-1*x);		//reversed to make the electronics the front of the robot
	double yIn = DeadZone(-1*y);
	double rotation = DeadZone(-1*r);

	//yIn = -yIn;
	xIn = -xIn;

	this->RotateVector(xIn, yIn, gyro);

	double wheelSpeeds[4];
	
	//Miss you crazy rascals - Tommy xoxo
	
	wheelSpeeds[0] = -1 * ( xIn + yIn + rotation);
	wheelSpeeds[1] =  1 * ( -xIn + yIn - rotation);
	wheelSpeeds[2] = -1 * (-xIn + yIn + rotation);
	wheelSpeeds[3] =  1 * ( xIn + yIn - rotation);

	this->Normalize(wheelSpeeds);
	
	for (int i=0; i<4; i++) //loop that disables and resets pid's when the speeds
	{						//are in the dead zone based for the individual pid
		if (::fabs(wheelSpeeds[i]) < Config::GetSetting("wheel_deadzone", .03))
		{
			//cout<<"Disabled speed "<<i<<endl;
			switch (i) {
				case 0:
					FLMotor->DisablePID();
					FLMotor->ResetPID();
					break;
				case 1:
					FRMotor->DisablePID();
					FRMotor->ResetPID();
					break;
				case 2:
					RLMotor->DisablePID();
					RLMotor->ResetPID();
					break;
				case 3:
					RRMotor->DisablePID();
					RRMotor->ResetPID();
					break;
			}
		}
	}
	
	double FLsetpoint = wheelSpeeds[0]*MAXSPEED;
	double FRsetpoint = wheelSpeeds[1]*MAXSPEED;
	double RLsetpoint = wheelSpeeds[2]*MAXSPEED;
	double RRsetpoint = wheelSpeeds[3]*MAXSPEED;
	
	if(halfSpeed)
	{
		FLMotor->Set(FLsetpoint * .15);
		FRMotor->Set(FRsetpoint * .15);
		RLMotor->Set(RLsetpoint * .15);
		RRMotor->Set(RRsetpoint * .15);
	}
	else if(!speedModify)
	{
		FLMotor->Set(FLsetpoint*SPEEDMODIFY);
		FRMotor->Set(FRsetpoint*SPEEDMODIFY);
		RLMotor->Set(RLsetpoint*SPEEDMODIFY);
		RRMotor->Set(RRsetpoint*SPEEDMODIFY);
	}
	else
	{
		FLMotor->Set(FLsetpoint);
		FRMotor->Set(FRsetpoint);
		RLMotor->Set(RLsetpoint);
		RRMotor->Set(RRsetpoint);
	}
	
}
void DriveTrain::Normalize(double *wheelSpeeds)
{	//Changes the wheel speeds to be in a scale from -1 to 1
	double maxMagnitude = fabs(wheelSpeeds[0]);
			INT32 i;
			for (i=1; i<4; i++)
			{
				double temp = fabs(wheelSpeeds[i]);
				if (maxMagnitude < temp) maxMagnitude = temp;
			}
			if (maxMagnitude > 1.0)
			{
				for (i=0; i<4; i++)
				{
					wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
				}
			}	

}
void DriveTrain::logHeaders(ostream &f)
{
	char * motor_names[4] = { "FrontLeft", "FrontRight", "RearLeft", "RearRight" };
	char * header_names[3] = { "Setpoint","Output","Current"};
	
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			f << motor_names[i] << " " << header_names[j] << ","; 
		}
	}

}
void DriveTrain::log(ostream &f)
{
	
	//log for each module
	
	SpeedControlTalon * motors[4] = { FLMotor, FRMotor, RLMotor, RRMotor };
	CurrentSensor * sensors[4] = { FLSensor, FRSensor, RLSensor, RRSensor };  
	for (int i = 0; i < 4; i++) {
		f << motors[i]->GetSetpoint() << "," <<  motors[i]->GetOutput() << ",";
		f << sensors[i]->GetCurrent() << ",";
	}
	
}

void DriveTrain::resetDriveMotors()
{
	/*
	setUpSpeedControl(FLMotor);
	setUpSpeedControl(FRMotor);
	setUpSpeedControl(RLMotor);
	setUpSpeedControl(RRMotor);
	
	cout<<"Drive kp: " << FLMotor->GetP() << endl;
	cout<<"Drive ki: " << FLMotor->GetI() << endl;
	cout<<"Drive kd: " << FLMotor->GetD() << endl;
*/
}

float DriveTrain::DeadZone(float input) {
	if (::fabs(input) <= 0.05)return 0;
	return input;
}

