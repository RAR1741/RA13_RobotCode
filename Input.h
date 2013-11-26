#include "WPIlib.h"
class Input
{
public:
	Input();
	~Input();
	
	void collectData();
	bool getLeftstick(int bNum);
	bool getRightstick(int bNum);
	bool getThirdstick(int bNum);
private:
	Joystick* left;
	Joystick* right;
	Joystick* third;
	bool* leftStick[];
	bool* rightStick[];
	bool* thirdStick[];
};
