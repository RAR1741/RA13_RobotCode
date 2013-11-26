#include "WPILib.h"
#include "config.h"
#ifndef SpeedControlTalon_H__
#define SpeedControlTalon_H__

#define LINESPERREV (Config::GetSetting("drive_encoderlines",250))
#define m_MAXSPEED (Config::GetSetting("drive_max_speed",5000))

class SpeedControlTalon : SpeedController {
	public:
	SpeedControlTalon(int TalonPWMChannel, int encoderA, int encoderB);
		void SetPID(double p, double i, double d);
		virtual void Set(float target);
		virtual void Set(float target, UINT8 group) { tal->Set(target, group); }
		virtual void Disable() { tal->Disable(); }
		
		virtual float Get() { return tal->Get(); }
		void EnablePID();
		void DisablePID();
		void ResetPID();
		
		//Accessors
		double GetP();
		double GetI();
		double GetD();
		
		float GetSetpoint() { return pid->GetSetpoint(); }
		float GetOutput() { return encoder->PIDGet(); }
		
		virtual void PIDWrite(float v) { Set(v); }
	private:
		//PID object
		PIDController* pid;
		
		//PID source object
		Encoder* encoder;
		//PID output
		Talon* tal;
	};
	#endif
