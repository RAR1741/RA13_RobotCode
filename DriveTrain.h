#include "WPIlib.h"
#include "Config.h"
#include "SpeedControlTalon.h"
#include "CurrentSensor.h"
#include <fstream>
#ifndef DRIVE_H__
#define DRIVE_H__

#define DRIVE_KP (Config::GetSetting("drive_kp",0.000025))
#define DRIVE_KI (Config::GetSetting("drive_ki",0.0000010))
#define DRIVE_KD (Config::GetSetting("drive_kd",0.000040))
#define CONTROLMODE (Config::GetSetting("control_mode",2))
#define MAXSPEED (Config::GetSetting("drive_max_speed",5000))
#define RATELIMITAMOUNT (Config::GetSetting("drive_ratelimit_perTick",5000))
#define SPEEDMODIFY (Config::GetSetting("drive_speed_modify",0.5))

class DriveTrain
{
public:
	//Constructors and deconstructors
	DriveTrain(int fl, int fr, int rl, int rr);
	~DriveTrain();
	
	//Drive functions
	void Drive(double x, double y, double r, double gyro, bool speedModify,bool halfSpeed);
	void RotateVector(double &x, double &y, double angle);
	void logHeaders(ostream &f);
	void log(std::ostream &f);
	void resetDriveMotors();
	
	void EnablePID() { FLMotor->EnablePID(); FRMotor->EnablePID(); RLMotor->EnablePID(); RRMotor->EnablePID(); }
	void DisablePID() { FLMotor->DisablePID(); FRMotor->DisablePID(); RLMotor->DisablePID(); RRMotor->DisablePID(); }
	void ResetPID() { FLMotor->ResetPID(); FRMotor->ResetPID(); RLMotor->ResetPID(); RRMotor->ResetPID(); }
	void SetPID(double p, double i, double d) { FLMotor->SetPID(p,i,d); FRMotor->SetPID(p,i,d); RLMotor->SetPID(p,i,d); RRMotor->SetPID(p,i,d); }
	
	void setGyroDrive(bool b);
private:
	//Robot variables
	RobotDrive * m_drive;
		
	SpeedControlTalon * FLMotor;
	SpeedControlTalon * FRMotor;
	SpeedControlTalon * RLMotor;
	SpeedControlTalon * RRMotor;
	CurrentSensor      * FLSensor;
	CurrentSensor      * FRSensor;
	CurrentSensor      * RLSensor;
	CurrentSensor      * RRSensor;
	
	bool gyroDrive; // true = field orientation , false = robot orientation
	
	//Private utility drive functions
	void Normalize(double *wheelSpeeds);
	float DeadZone(float input);
	
};
#endif
