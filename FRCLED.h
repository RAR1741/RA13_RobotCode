#include "WPILib.h"
#include "config.h"

#define triangle Relay::kOn
#define circle Relay::kForward
#define square Relay::kReverse
#define off Relay::kOff
#define DELAY (Config::GetSetting("lights_delay",100))

class FRCLED
{
public:
	FRCLED(int spike1, int spike2,int spike3);
	void pattern1();
	void pattern2();
	void pattern3();
	void pattern4();
	void resetTime();
private:
	int time;
	int state;
	Relay* s1;
	Relay* s2;
	Relay* s3;
};
