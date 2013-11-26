#include "Input.h"

Input::Input()
{
	left = new Joystick(1);
	right = new Joystick(2);
	third = new Joystick(3);
	*leftStick = new bool[12];
	*rightStick = new bool[12];
	*thirdStick = new bool[12];
}
Input::~Input()
{
	
}

/*!
 * Collect data for all joysticks only once
 */
void Input::collectData()
{
	for(int i=0; i<12; i++)
	{
		*leftStick[i] = left->GetRawButton(i+1);
		*rightStick[i] = right->GetRawButton(i+1);
		*thirdStick[i] = third->GetRawButton(i+1);
	}
}
/*!
 * Get left stick data
 */
bool Input::getLeftstick(int bNum)
{
	return *leftStick[bNum-1];
}
/*!
 * Get right stick data
 */
bool Input::getRightstick(int bNum)
{
	return *rightStick[bNum-1];
}
/*!
 * Get thrid stick
 */
bool Input::getThirdstick(int bNum)
{
	return *thirdStick[bNum-1];
}
