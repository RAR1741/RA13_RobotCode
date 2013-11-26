#include "WPILib.h"

class lightController
{
public:
	lightController(int jag1, int jag2);
	void useLights();
	void setMode(int m);
	void holdState(bool b);
private:
	double cosWave(int p) {return lightController::cosWave(p,0);}
	double cosWave(int p, int o);
	int counter;
	bool holdCurrentState;
	CANJaguar* light1;
	CANJaguar* light2;
	int lightState;
	int light1Value, light2Value;
};
