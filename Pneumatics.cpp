#include "Pneumatics.h"

Pneumatics::Pneumatics(int collector_a, int collector_b, int endgame_a, int endgame_b)
{
	collector = new DoubleSolenoid(collector_a, collector_b);
	endgame   = new DoubleSolenoid(endgame_a, endgame_b);
	
	cooldown_collector = new Timer();
	cooldown_endgame = new Timer();
}

/**
 * @brief Handles a generic subsystem. Sets the solenoid s to the value val if the cooldown period has elapsed.
 * @remarks Also resets the timer if that period has elapsed
 */
void Pneumatics::SetWithCooldown(DoubleSolenoid * s, Timer * t, float cooldown, DoubleSolenoid::Value val)
{
	// Handle the timer first. If we *just* passed the cooldown period, we'll still want to continue.
	
	// Several possible states:
	if (t->HasPeriodPassed(cooldown)) {
		t->Stop();
		t->Reset();
	}
	DoubleSolenoid::Value oldVal = s->Get();
	
	if (oldVal != val) {
		// A change is necessary!
		
		// Timer is not running. 
		// TODO: Timer could use a function to say whether it's running or not.
		if (t->Get() == 0) {
			t->Start();
			s->Set(val);
		}
	} else {
		s->Set(val); // This should only be necessary if the solenoid doesn't need "pulsing"
	}
}
void Pneumatics::Process()
{
	
}
void Pneumatics::extendEndgame()
{
	endgame->Set(DoubleSolenoid::kForward);
}
void Pneumatics::retractEndgame()
{
	endgame->Set(DoubleSolenoid::kReverse);
}
void Pneumatics::extendCollector()
{
	collector->Set(DoubleSolenoid::kForward);
}
void Pneumatics::retractCollector()
{
	collector->Set(DoubleSolenoid::kReverse);
}
