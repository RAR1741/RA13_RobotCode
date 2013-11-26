#include <wpilib.h>
#include <fstream>
#include "Config.h"
#include <cmath>
#ifndef SHOOTER_H
#define SHOOTER_H

#define WHEEL_TOLERANCE (Config::GetSetting("auto_shooter_tolerance", 50))
#define ANGLE_BIAS      (Config::GetSetting("shooter_angle_bias",0))

class Shooter
{
	public:
	Shooter(int mainWheel,int secondaryWheel,int angleChannel,int LoaderID, int home_id);
	Shooter(int mainWheel,int secondaryWheel,int angleChannel,int LoaderID);
	~Shooter();
	
	void ChamberFrisbee(float power = -0.75);
	void StopLoader();
	void ResetCANJaguars();
	void logHeaders(std::ostream &f);
	void log(std::ostream &f);
	
	void SetAngle(float angle);
	void SetSpeeds(float mspeed, float sspeed);
	
	float GetAngle() { 
		return RotationsToAngle(AngleJag->GetPosition()) - ANGLE_BIAS;
	}
	bool IsAtTargetAngle();
	bool IsReady() { return m_state == READY; }
	bool IsFrontWheelAtTargetSpeed() { return ::fabs( MainWheel->GetSpeed() - MainWheel->Get()) <= WHEEL_TOLERANCE; }
	bool IsRearWheelAtTargetSpeed() { return ::fabs( SecondaryWheel->GetSpeed() - SecondaryWheel->Get()) <= WHEEL_TOLERANCE; } 
	void Home();
	void ManualControl(float front, float rear, float updown, float loader);
	void ManualAngleControl(float desired_position);
	
	float getFrontWheelTarget() { return MainWheel->Get(); }
	float getRearWheelTarget() { return SecondaryWheel->Get(); }
	double getFrontWheelSpeed();
	double getRearWheelSpeed();
	
	int getFiringState();
	
	// This is the master control process.
	// Needs to be called every loop.
	void Process();
	void ResetShooterProcess(void);
	
	void ResetShooterMotors();
	
	private:
	typedef enum {
		START = 0,					// Starts in this mode, un-homed
		HOMING_DOWN = 1,    		// Homing downwards. 
		HOMING_TURNAROUND = 2,		// Met homing switch 
		HOMING_UP = 3,      		// Homing upwards.
		READY = 4,					// Ready for commands
		ANGLING = 5,				// Shifting to new angle
		FIRING = 6,					// in process of firing
		MANUAL = 7,					// Manual override
	} ShooterState;
	
	void ResetAngleControl();
	static float AngleToRotations(float theta);
	static float RotationsToAngle(float rotations);
	
	bool IsHome() {
		return HomeSwitch->Get() > 0;
	}
	
	bool IsAtDownLimit() {
		return !AngleJag->GetReverseLimitOK();
	}
	
	bool IsAtUpLimit() {
		return !AngleJag->GetForwardLimitOK();
	}
	
	bool ReadyForCommands() {
		return m_state == Shooter::READY;
	}
	
	
	// Methods for self-testing formulas.
	// void AutoTest();
	
	// constants for the rotations/angle formula
	static const float kA;
	static const float kB;
	static const float kC;
	static const float kD;
	static const float kPI; 
	
	// Hardware
	CANJaguar * MainWheel;
	CANJaguar * SecondaryWheel;
	CANJaguar * Loader;
	CANJaguar * AngleJag;
	DigitalInput * HomeSwitch;
	
	// State information
	Timer * m_turnaround;
	int m_state;
	float m_desired_angle;
	float m_desired_angle_in_rotations;

};
#endif
