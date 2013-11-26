#ifndef GAMEPAD_H_
#define GAMEPAD_H_
#include <stdio.h>

class DriverStation;

/**
 * Handle input from Logitech Dual Action Gamepad connected to the Driver
 * Station.
 */
class Gamepad
{
public:
    typedef enum
    {
        kLeftXAxis, kLeftYAxis, kRightXAxis, kRightYAxis
    } AxisType;

    typedef enum
    {
        kCenter, kUp, kUpLeft, kLeft, kDownLeft, kDown, kDownRight, kRight,
        kUpRight
    } DPadDirection;

    Gamepad(UINT32 port);
    ~Gamepad();

    float GetLeftX();
    float GetLeftY();
    float GetRightX();
    float GetRightY();
    float GetAxis(AxisType axis);
    float GetRawAxis(UINT32 axis);

    bool GetNumberedButton(unsigned buttonNumber);
    bool GetLeftPush(); 
    bool GetRightPush();
    bool GetA() { return GetNumberedButton(2); }
    bool GetB() { return GetNumberedButton(3); }
    bool GetY() { return GetNumberedButton(4); }
    bool GetX() { return GetNumberedButton(1); }
    bool GetLeftBumper() { return GetNumberedButton(5); }
    bool GetRightBumper() { return GetNumberedButton(6); }
    bool GetBack() { return GetNumberedButton(9); }
    bool GetStart() { return GetNumberedButton(10); }
    bool GetLeftTrigger() { return GetNumberedButton(7); }
    bool GetRightTrigger() { return GetNumberedButton(8); }
    

    DPadDirection GetDPad();

protected:
    static const UINT32 kLeftXAxisNum = 1;
    static const UINT32 kLeftYAxisNum = 2;
    static const UINT32 kRightXAxisNum = 3;
    static const UINT32 kRightYAxisNum = 4;
    static const UINT32 kDPadXAxisNum = 5;
    static const UINT32 kDPadYAxisNum = 6;

    static const unsigned kLeftAnalogStickButton = 11;
    static const unsigned kRightAnalogStickButton = 12;

    DriverStation *ap_ds;
    UINT32 a_port;
};

#endif // GAMEPAD_H_

