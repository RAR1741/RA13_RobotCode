#include "Shooter.h"
#include "Config.h"
#include <iostream>
#include <cmath>

#define HOME_POSITION_SHAFT_LENGTH (Config::GetSetting("home_position_shaft_length", 9.375))

using std::cout;
using std::endl;

const float Shooter::kA = 9.875;
const float Shooter::kB = 9.75;
const float Shooter::kC = 2.25;
const float Shooter::kD = 2.125;
const float Shooter::kPI = 3.14159265358979;

Shooter::Shooter(int mainWheel,int secondaryWheel, int angleChannel, int LoaderID, int HomeID)
{
	CANJaguar::ControlMode mode = CANJaguar::kPercentVbus;
	MainWheel = new CANJaguar(mainWheel,CANJaguar::kSpeed);
	SecondaryWheel = new CANJaguar(secondaryWheel,CANJaguar::kSpeed);
	Loader = new CANJaguar(LoaderID,mode);
	AngleJag = new CANJaguar(angleChannel, CANJaguar::kPosition);
	HomeSwitch = new DigitalInput(HomeID);
	m_turnaround = new Timer();
	m_state = Shooter::START;
	
	MainWheel->ChangeControlMode(CANJaguar::kSpeed);
	MainWheel->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
	MainWheel->ConfigEncoderCodesPerRev(100);
	MainWheel->SetPID(Config::GetSetting("shooter_kp", -0.25),
							  Config::GetSetting("shooter_ki", -0.005),
							  Config::GetSetting("shooter_kd", -0.0001));
	MainWheel->EnableControl(0);
	
	SecondaryWheel->ChangeControlMode(CANJaguar::kSpeed);
	SecondaryWheel->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
	SecondaryWheel->ConfigEncoderCodesPerRev(100);
	SecondaryWheel->SetPID(Config::GetSetting("rear_kp", -0.25),
							 Config::GetSetting("rear_ki", -0.005),
							 Config::GetSetting("rear_kd", -0.0001));
	SecondaryWheel->EnableControl(0);
	
	this->ResetAngleControl();
}



void Shooter::ResetAngleControl() {
	double angle_p = Config::GetSetting("lever_kp", 20);
	double angle_i = Config::GetSetting("lever_ki", 0);
	double angle_d = Config::GetSetting("lever_kd", 0);
	AngleJag->ChangeControlMode(CANJaguar::kPosition);
	AngleJag->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
	AngleJag->SetPID(angle_p,angle_i,angle_d);
	AngleJag->ConfigEncoderCodesPerRev(100);
	//AngleJag->ConfigSoftPositionLimits(-5000, 5000);
	AngleJag->EnableControl(0);
}

float Shooter::AngleToRotations(float theta) //Assumes the shooter has been zero'ed at home position first
{
	float theta_in_rads = kPI * theta / 180;
	float kBeta = ::atan(kC/kB); 
	float kF = ::sqrt(kB * kB + kC * kC);
	float kE = ::sqrt((kA * kA) + (kF * kF) - (2*kA*kF*(::cos(kPI / 2.0 - kBeta - theta_in_rads)))    );
	float kX = ::sqrt( (kE * kE) - (kD * kD));
	float rotations =  ( (kX - HOME_POSITION_SHAFT_LENGTH) * 16 * 18) * -1 ;
	//cout << "Beta = " << kBeta << " F = " << kF << " E = " << kE << " X = " << kX << " commanded = " << rotations << endl;
	return rotations;
}

float Shooter::RotationsToAngle(float rotations)
{
	float kBeta = ::atan(kC/kB); 
	float kF = ::sqrt(kB * kB + kC * kC);
	float kX = (- rotations/ (16*18)) + HOME_POSITION_SHAFT_LENGTH;
	float theta = (::asin( ( (kA * kA) + (kF * kF) - (kD * kD) - (kX * kX) ) / (2 * kA * kF))) - kBeta;
	float theta_in_degrees = theta * 180.0 / kPI;
	return theta_in_degrees;
}

void Shooter::SetSpeeds(float mspeed, float sspeed) // (RPM,RPM)
{
	MainWheel->Set(mspeed);
	SecondaryWheel->Set(sspeed);
}

void Shooter::Process()
{
	//cout << "Shooter Process: We started the loop in state " << m_state << endl;
	
	switch(m_state) {
	case Shooter::START:
		// Start homing.
		// Initialize the shooter to voltage control mode.
		AngleJag->ChangeControlMode(CANJaguar::kPercentVbus);
		//cout << "In start state... getting ready to go up..." << endl;
		m_turnaround->Reset();
		m_turnaround->Start();
		m_state = Shooter::HOMING_TURNAROUND;
		break;
	case Shooter::HOMING_DOWN:
		// We're moving down.
		AngleJag->Set(Config::GetSetting("shooter_homing_down_speed", -0.5));
		
		// If we hit the home switch...
		if (IsHome()) {
			// We're going in the wrong direction. We want to finish homing while moving up. Turn around after a turnaround time,
			m_turnaround->Reset();
			m_turnaround->Start();
			m_state = Shooter::HOMING_TURNAROUND;
			break;
		} else if (IsAtDownLimit()) {
			// We hit the bottom, this shouldn't happen? because we always start homing up first...
			m_state = Shooter::HOMING_UP;
			//cout << "Was \"homing down\" and hit the down limit, going up..." << endl;
			break;
		} else if (IsAtUpLimit()){
			//Just in case someting weird happens
			m_state = Shooter::HOMING_DOWN;
			//cout << "Was \"homing down\" and hit the up limit, going down..." << endl;
			break;
		}
		
		break;
	case Shooter::HOMING_UP:
		// We're going up.
		AngleJag->Set(Config::GetSetting("shooter_homing_up_speed", 0.5));
		
		if (IsHome()) {
			// HOLD IT. 
			AngleJag->Set(0);
			ResetAngleControl();
			m_state = Shooter::READY;
		} else if (IsAtUpLimit()) {
			// Hit upper limit. This would happen if the shooter started angled too high
			m_state = Shooter::HOMING_DOWN;
			//cout << "Was \"homing up\" and hit the up limit, going down..." << endl;
			break;
		} else if (IsAtDownLimit()) {
			//Just in case someting weird happens
			m_state = Shooter::HOMING_UP;
			//cout << "Was \"homing up\" and hit the down limit, going up..." << endl;
			break;
		}
		
		break;
	case Shooter::HOMING_TURNAROUND:
		// Keep going in the same direction for a while
		
		// Turnaround will only be entered while traveling down, so continue spinning down
		AngleJag->Set(Config::GetSetting("shooter_homing_down_speed", -0.5));
		
		
		if (m_turnaround->HasPeriodPassed( Config::GetSetting("shooter_homing_turnaround_time", 2) )) {
			m_turnaround->Stop();
			m_state = Shooter::HOMING_UP;
			break;
		} else if (IsAtUpLimit()) { // If we hit a limit, reverse.
			m_state = Shooter::HOMING_DOWN;
			break;
		} else if (IsAtDownLimit()) {
			m_state = Shooter::HOMING_UP;
			break;
		}
		break;
	case Shooter::READY:
		// We aren't doing anything! Cool!
		break;
	case Shooter::ANGLING:
		m_desired_angle_in_rotations = AngleToRotations(m_desired_angle);
		AngleJag->Set(m_desired_angle_in_rotations);
		
		if(IsAtUpLimit() || IsAtDownLimit()) //If we hit a limit, the angle checking won't work right so we need to just exit
		{
			m_state = Shooter::READY;
		}

		if (IsAtTargetAngle()) {
			m_state = Shooter::READY;
			break;
		}
		
		break;
		
	case Shooter::FIRING:
		// Not currently important.
		m_state = Shooter::READY;
		break;
	}
	
	// cout << "Shooter Process: We ended the loop in state " << m_state << endl;
}

void Shooter::ResetShooterProcess(void)
{
	ResetCANJaguars();
	m_state = Shooter::START;
}

void Shooter::SetAngle(float angle)
{
	if (ReadyForCommands()) {
		m_desired_angle = angle + ANGLE_BIAS;
		m_state = Shooter::ANGLING;
	}
}

bool Shooter::IsAtTargetAngle() {
	// Check to see if we're in position control
	if (AngleJag->GetControlMode() != CANJaguar::kPosition) {
		// We're not! This question is meaningless.
		return false;
	} 
	float angleerror = (::fabs(m_desired_angle_in_rotations - AngleJag->GetPosition()) );
	// cout << "Angle Error = " << angleerror << endl;
	return ( angleerror < Config::GetSetting("shooter_angle_tolerance", 10));
}

void Shooter::ChamberFrisbee(float power) //Copyright Wham O
{
	Loader->Set(power);
}

int Shooter::getFiringState()
{
	if(::fabs(MainWheel->GetOutputVoltage())>.75)
		if(::fabs(Loader->GetOutputVoltage())>.75)
			return 2; // Wheels & loader
		else
			return 1; // Just wheels
	else
		return 0; // nothin'.
}

void Shooter::StopLoader()
{
	Loader->Set(0);
}

void Shooter::ManualControl(float front, float rear, float updown, float loader) // (RPM,RPM,Percentage,Percentage)
{
	m_state = Shooter::MANUAL;
	if (AngleJag->GetControlMode() != CANJaguar::kPercentVbus) {
		AngleJag->ChangeControlMode(CANJaguar::kPercentVbus);
	}
	MainWheel->Set(front);
	SecondaryWheel->Set(rear);
	AngleJag->Set(updown);
	Loader->Set(loader);
}

void Shooter::ManualAngleControl(float desired_angle) // (theta)
{
	//ASSUMES THE MOTOR HAS BEEN ZERO'ED AT HOME POSITION FIRST
	float rotations = this->AngleToRotations(desired_angle);
	AngleJag->Set(rotations);
}

void Shooter::ResetShooterMotors()
{
	MainWheel->ChangeControlMode(CANJaguar::kSpeed);
	MainWheel->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
	MainWheel->ConfigEncoderCodesPerRev(100);
	MainWheel->SetPID(Config::GetSetting("shooter_kp", -0.25),
							  Config::GetSetting("shooter_ki", -0.005),
							  Config::GetSetting("shooter_kd", -0.0001));
	MainWheel->EnableControl(0);
	
	SecondaryWheel->ChangeControlMode(CANJaguar::kSpeed);
	SecondaryWheel->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
	SecondaryWheel->ConfigEncoderCodesPerRev(100);
	SecondaryWheel->SetPID(Config::GetSetting("rear_kp", -0.25),
							 Config::GetSetting("rear_ki", -0.005),
							 Config::GetSetting("rear_kd", -0.0001));
	SecondaryWheel->EnableControl(0);
}

void Shooter::ResetCANJaguars()
{	
	this->ResetShooterMotors();
	this->ResetAngleControl();
	m_state = Shooter::START;
}


void Shooter::logHeaders(ostream &f)
{
	char * motor_names[4] = { "Main", "Secondary", "Angle", "Loader" };
	char * fields[5] = {"Speed","Position","Setpoint","Voltage","Current"};
	
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 5; j++) {
			f << motor_names[i] << ' ' << fields[j] << ',';
		}
	}
	f << "Shooter State,Desired Angle,IsLowerLimit,IsUpperLimit,IsHome,";
 	//f << "Main Speed,Main Setpoint,Main Voltage,Main Current,Secondary Speed,Secondary Setpoint,Secondary Voltage,Secondary Current,";
}

void Shooter::log(ostream &f)
{
	CANJaguar * motors[4] = {MainWheel,SecondaryWheel,AngleJag,Loader};
	for (int i = 0; i < 4; i++) {
		f << motors[i]->GetSpeed() << ',';
		f << motors[i]->GetPosition() << ',';
		f << motors[i]->Get() << ',';
		f << motors[i]->GetOutputVoltage() << ',';
		f << motors[i]->GetOutputCurrent() << ',';
	}
	
	//f << MainWheel->GetSpeed() << "," << MainWheel->Get() << "," << MainWheel->GetOutputVoltage() << "," << MainWheel->GetOutputCurrent() << "," <<
	//		SecondaryWheel->GetSpeed() << "," << SecondaryWheel->Get() << "," << SecondaryWheel->GetOutputVoltage() << "," << SecondaryWheel->GetOutputCurrent() << ",";
	f << m_state << ',' << m_desired_angle << ',' << IsAtDownLimit() << ',' << IsAtUpLimit() << ',' << IsHome() << ',';
}

double Shooter::getFrontWheelSpeed()
{
	return MainWheel->GetSpeed();
}
double Shooter::getRearWheelSpeed()
{
	return SecondaryWheel->GetSpeed();
}

Shooter::~Shooter()
{
	
}
