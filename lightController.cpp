#include "lightController.h"
#include <cmath>

using namespace std;

lightController::lightController(int jag1, int jag2)
{
	light1 = new CANJaguar(jag1);
	light2 = new CANJaguar(jag2);
	light1Value = 0;
	light2Value = 0;
	counter = 0;
	lightState = 0;
	holdCurrentState = false;
}
void lightController::setMode(int m)
{
	if(!holdCurrentState)
		lightState = m;
}
void lightController::useLights()
{
	++counter;
	switch(lightState)
	{
		case 1: // Weird side-side pulse
		{
			light1Value = 1;
			light2Value = 1;
		}
			break;
		case 2: // alternate on/off every 2 seconds
		{
			if((counter % 80)<40)
			{
				light1Value = 0;
				light2Value = 0;
			}
			else
			{
				light1Value = 1;
				light2Value = 1;
			}
		}
			break;
		case 3: // heartbeat mode, 2 seconds
		{
			int lightModulo = counter % 40;
			if(lightModulo<3)
			{
				light1Value = 1;
				light2Value = 1;
			}
			else if(lightModulo<6)
			{
				light1Value = 0;
				light2Value = 0;
			}
			else if(lightModulo<9)
			{
				light1Value = 1;
				light2Value = 1;
			}
			else
			{
				light1Value = 0;
				light2Value = 0;
			}
		}
			break;
		case 4: // ramp up, restart from bottom
		{
			double rampValue = (counter % 21) * 0.05;
			light1Value = static_cast<int>(rampValue);
			light2Value = static_cast<int>(rampValue);
		}
			break;
		case 5: // faster on/off flash 
		{
			if((counter % 6)<3)
			{
				light1Value = 1;
				light2Value = 1;
			}
			else
			{
				light1Value = 0;
				light2Value = 0;
			}
		}
		case 6: // faster on/off side-side flash 
		{
			if((counter % 6)<3)
			{
				light1Value = 1;
				light2Value = 0;
			}
			else
			{
				light1Value = 0;
				light2Value = 1;
			}
		}
			break;
		case 7: // Weird side-side pulse
		{
			light1Value = static_cast<int>(cosWave(counter,60));
			light2Value = static_cast<int>(cosWave(counter+30,60));
		}
			break;
		default:
		{
			light1Value = 0;
			light2Value = 0;
		}
			break;
	}
	light1->Set(light1Value);
	light2->Set(light2Value);
}
void lightController::holdState(bool b)
{
	holdCurrentState = b ; 
}

double lightController::cosWave(int p, int o )
{
	double xDegrees = 359 * ((p % o) / o);
	double xRadians = xDegrees * (3.14159 / 180);
	return 0.5*cos(xRadians) + 0.5;
}
