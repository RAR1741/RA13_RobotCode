#include "Climber.h"

Climber::Climber(int climber1_id, int climber2_id) {
	m_climber1 = new CANJaguar(climber1_id, CANJaguar::kPercentVbus);
	m_climber2 = new CANJaguar(climber2_id, CANJaguar::kPercentVbus);
}

void Climber::Drive(float speed, bool override)
{
	float limited_speed = speed;
	
	// Prevent back-drive unless override is true
	// Should be like two keys in a missle silo
	// or something similar to prevent accidental
	// damage to the mechanism.
	if (limited_speed < 0.0 && ! override) {
		limited_speed = 0;
	}
	
	m_climber1->Set(limited_speed);
	m_climber2->Set(limited_speed);
}

void Climber::LogHeaders(ostream & f) {
	char * motor_names[2] = { "Climber 1", "Climber 2" };
	char * fields[3] = {"Setpoint","Voltage","Current"};
	
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 3; j++) {
			f << motor_names[i] << ' ' << fields[j] << ',';
		}
	}
}

void Climber::Log(ostream &f)
{
	CANJaguar * motors[2] = { m_climber1, m_climber2 };
	for (int i = 0; i < 2; ++i) {
		f << motors[i]->Get() << ',';
		f << motors[i]->GetOutputVoltage() << ',';
		f << motors[i]->GetOutputCurrent() << ',';
	}
}

