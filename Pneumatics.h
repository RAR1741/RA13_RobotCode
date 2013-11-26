#ifndef PNEUMATICS_H__
#define PNEUMATICS_H__
#include "WPILib.h"
#include "Config.h"
#include <iostream>

class Pneumatics {
public:
	Pneumatics(int collector_a, int collector_b, int endgame_a, int endgame_b);
	void Process();
	
	void extendEndgame();
	void retractEndgame();
	void extendCollector();
	void retractCollector();
	
	void Log(ostream & out);
	void LogHeaders(ostream & out);
private:
	void SetWithCooldown(DoubleSolenoid * s, Timer * t, float cooldown, DoubleSolenoid::Value val);
	
	DoubleSolenoid * collector;
	DoubleSolenoid * endgame;
	Timer * cooldown_collector;
	Timer * cooldown_endgame;
};

#endif
