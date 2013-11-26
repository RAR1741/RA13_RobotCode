#include "WPILib.h"
#include "config.h"
#ifndef SpeedControlVictor_H__
#define SpeedControlVictor_H__

#define LINESPERREV (Config::GetSetting("drive_encoderlines",250))
#define m_MAXSPEED (Config::GetSetting("drive_max_speed",5000))

class SpeedControlVictor : SpeedController {
	public:
	SpeedControlVictor(int VictorPWMChannel, int encoderA, int encoderB);
		void SetPID(double p, double i, double d);
		virtual void Set(float target);
		virtual void Set(float target, UINT8 group) { vic->Set(target, group); }
		virtual void Disable() { vic->Disable(); }
		
		virtual float Get() { return vic->Get(); }
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
		Victor* vic;
	};
	#endif
