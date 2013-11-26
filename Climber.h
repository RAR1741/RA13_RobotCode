#ifndef CLIMBER_H__
#define CLIMBER_H__

#include "wpilib.h"

class Climber {
public:
	Climber(int climber1_id, int climber2_id);
	
	void Drive(float speed, bool override);
	
	void LogHeaders(ostream & f);
	void Log(ostream & f);
private:
	CANJaguar * m_climber1;
	CANJaguar * m_climber2;
};

#endif
