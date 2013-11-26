#include "WPILib.h"
#include "fletcher.h"
#include "DriveTrain.h"
#include "Gamepad.h"
#include "Scoop.h"
#include "Shooter.h"
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <cmath>
#include "Config.h"
//#include "Target.h"
#include "Climber.h"
#include "CurrentSensor.h"
#include "networktables2/type/NumberArray.h"
#include "MagicBox.h"
#include "Compressor.h"
#include "Pneumatics.h"
#include "lightController.h"
//#include "TargetServer.h"

#define KP (Config::GetSetting("drive_kp",0.000025))
#define KI (Config::GetSetting("drive_ki",0.0000010))
#define KD (Config::GetSetting("drive_kd",0.000040))

#define SHOOTER_FRONTWHEEL_MANUAL (Config::GetSetting("shooter_front_wheel_manual",1.0)) //Percentage
#define SHOOTER_REARWHEEL_MANUAL (Config::GetSetting("shooter_rear_wheel_manual",0.5)) //Percentage
#define SHOOTER_FRONTWHEEL_SPEED (Config::GetSetting("shooter_main_speed",2000)) //RPM
#define SHOOTER_REARWHEEL_SPEED (Config::GetSetting("shooter_secondary_speed",2000)) //RPM

#define AUTONOMOUS_PROGRAM (Config::GetSetting("autoProgram",1))
#define AUTONOMOUS_FRONT_WHEEL_SPEED (Config::GetSetting("auto_Front_Wheel_Speed",2000))
#define AUTONOMOUS_REAR_WHEEL_SPEED (Config::GetSetting("auto_Rear_Wheel_Speed",1000))
#define AUTONOMOUS_SPEED_TOLERANCE (Config::GetSetting("auto_Shooter_Tolerance",50))
#define AUTO_DELAY (Config::GetSetting("auto_delay",5))
#define AUTO_ANGLE (Config::GetSetting("auto_angle",23.5))

using namespace std;

class RA13Robot : public IterativeRobot
{
private:
	Gamepad* driverGamepad;
	Gamepad* operatorGamepad;
	//Gyro* driveGyro;
	MagicBox* magicBox;
	DriveTrain* myDrive;
	ofstream fout;
	Scoop* myScoop;
	Shooter* myShooter;
	//Climber * myClimber;
	Relay * cameraLight;
	Compressor * compressor;
	Pneumatics* pneumatics;
	//TargetServer * ts;
	
	int target_state;
	bool toggle;
	int LEDTimer;
	
	bool autoReady;
	lightController* lc;
	//CANJaguar* light1;
	//CANJaguar* light2;
	
	
	char shooter_front_latch;
	char shooter_rear_latch;
	
	Gamepad::DPadDirection lastdir;
	
	float front_setpoint;
	float rear_setpoint;
	
	typedef enum { 
		READY = 0,
		MOVING = 1,
		DONE = 2,
	} TargetState;
	
	//Autonomous variables
	int autoState;
	double autoDelay;
	Timer * autoSpinUpTimer;
	float manualAngle;
	float savedAngle; // For keeping track of old angle
					  // when crouching
	bool justCrouched;
	
	Timer * logTimer;

	
	bool freshStart; // True if robot *just* turned on
	
	DriverStation * ds;
	
	int CurrentMode() {
		if (this->IsDisabled()) {
			return 0;
		} else if (this->IsAutonomous()) {
			return 1;
		} else if (this->IsOperatorControl()) {
			return 2;
		} else 
			return 3;
	}
public:
	RA13Robot(void)
	{
		myDrive = NULL;
		driverGamepad = NULL;
		operatorGamepad = NULL;
		myScoop = NULL;
		myShooter = NULL;
		//myClimber = NULL;
		//driveGyro = NULL;
		magicBox = NULL;
		shooter_front_latch = 0;
		shooter_rear_latch = 0;
		
		pneumatics = NULL;
		
		front_setpoint = rear_setpoint = 0;
		target_state = RA13Robot::READY;
		
		autoState = 0;
		autoDelay = 0;
		
		autoSpinUpTimer = new Timer();
		
		manualAngle = 0;
		savedAngle = 0;
		justCrouched = false;
		
		logTimer = NULL;
		
		LEDTimer = 0;
		
		//light1 = NULL;
		//light2 = NULL;
		
		autoReady = false;
		
		freshStart = true;
		
		ds = NULL; 
		
		//ts = new TargetServer();
	}
	
	void RobotInit() { 
		GetChecksum();
		
		SmartDashboard::init();
		//SmartDashboard::PutString("TargetInfo","");
		//SmartDashboard::PutString("Potato","");
		cout << "2013 Red Alert Robot" << endl;
		cout << "Compiled on: ";
		cout << __DATE__ << " at " << __TIME__ << endl;
		
		//Initilize gamepads
		
		cout << "Initializing gamepads..." << endl;
		driverGamepad = new Gamepad(1);
		operatorGamepad = new Gamepad(2);
		cout << "Gamepads initialized." << endl;
		
		cout << "Initializing drive train..." << endl;
		//Initilize drive train
		myDrive = new DriveTrain(1,2,3,4);
		cout << "Drive train initialized." << endl;
		
		//Initialize scoop
		//myScoop = new Scoop(8);
		toggle = false;
		
		//Initialize shooter
		cout << "Initializing shooter..." << endl;
		myShooter = new Shooter(9,10,7,11,13);
		cout << "Shooter initialized." << endl;
		
		cout << "Initializing climber..." << endl;
		//myClimber = new Climber(12, 13);
		cout << "Climber initialized." << endl;
		
		cout << "Initializing gyro, please don't touch the robot" << endl;
		//driveGyro = new Gyro(1);
		magicBox = new MagicBox(1);
		cout << "Gyro initialized." << endl;
		
		cout << "Initializing light" << endl;
		cameraLight = new Relay(2, Relay::kForwardOnly);
		//PIDController1->SetTolerance(1);
		compressor = new Compressor(14, 1);
		cout << "Light initialized." << endl;
		
		cout<<"Initilizing pneumatics....";
		pneumatics = new Pneumatics(3,4,1,2);
		cout<<"Done!"<<endl;
		
		cout<<"Making pretty lights...";
		lc = new lightController(12,13);
		//light1 = new CANJaguar(12);
		//light2 = new CANJaguar(13);
		cout<<"Our robot is now pretty."<<endl;
		
		lastdir = Gamepad::kCenter;
		
		this->SetPeriod(0.05);
		
		OutputTroll(cout);
		callMe();
		SmartDashboard::init();
		ds = DriverStation::GetInstance();
		
		cout << "Starting misson timer..." << endl;
		logTimer = new Timer();
		logTimer->Reset();
		logTimer->Start();
		cout << "Started: " << logTimer->Get() << " seconds" << endl;
		cout << "          ##################################" << endl;
		cout << "          # >>> BOOT SEQUENCE COMPLETE <<< #" << endl;
		cout << "          ##################################" << endl;
		
		
	}
	void DisabledInit() { 
		//Config loading
		try {
			cameraLight->Set(Relay::kOff);
			if (!Config::LoadFromFile("config.txt")) {
				cout << "Something happened during the load." << endl;
			}
			Config::Dump();
			
			myDrive->DisablePID();
			myDrive->ResetPID();
			
			if(fout.is_open() && !freshStart && !ds->IsFMSAttached()){
				fout.close();
			myShooter->ResetShooterProcess();
			
			lc->holdState(false);
			}
		} catch (exception ex) {
			cout << "Disabled exception. Trying again." << endl;
			cout << "Exception: " << ex.what() << endl;
		}
		
		//ResetShooterMotors();
		/*
		SmartDashboard::PutNumber("Target Info S",1741);
		cout<<SmartDashboard::GetNumber("Target Info S");
		*/
	}
	void AutonomousInit() {
		autoDelay = 20 * AUTO_DELAY;
		if(!fout.is_open()) {
			fout.open("logging.csv");
			logheaders();
		}
			
		compressor->Start();
		autoState = 0;
		autoReady = false;
		lc->setMode(2);
		freshStart = false;
	} 
    void TeleopInit() {
    	//Open logging
    	if(!fout.is_open()) {
    		fout.open("logging.csv");
    		logheaders();
    	}
    	cameraLight->Set(Relay::kOff);
    	myDrive->SetPID(KP,KI,KD);
    	myDrive->EnablePID();
    	manualAngle = 0;
    	compressor->Start();
    	myDrive->setGyroDrive(true);
    	lc->setMode(0);
    	freshStart = false;
    	
    }
    void TestInit()	
    {
    	lc->setMode(3);
    	compressor->Start();
    	myShooter->ResetShooterProcess();
    }
	void DisabledPeriodic() { 
		//cout << "Packet: " <<  ts->GetLatestPacket() << endl;
	}
	void AutonomousPeriodic() {
		lc->useLights();
		switch( (int) AUTONOMOUS_PROGRAM)
		{
		case 1:
			switch(autoState)
			{
			case 0: //Waiting
				cout<<"Delay: " << autoDelay << endl;
				if(autoDelay > 0)
					autoDelay--;
				else/* if(autoReady = false)*/
					autoState = 3;
				//else
					//autoState = 1;
				break;
			case 1: //Spin up wheel speed and wait in this state until done
				
				myShooter->SetSpeeds(AUTONOMOUS_FRONT_WHEEL_SPEED,AUTONOMOUS_REAR_WHEEL_SPEED);
				if((myShooter->getFrontWheelSpeed() <= (AUTONOMOUS_SPEED_TOLERANCE + AUTONOMOUS_FRONT_WHEEL_SPEED) && myShooter->getFrontWheelSpeed() >= (AUTONOMOUS_FRONT_WHEEL_SPEED - AUTONOMOUS_SPEED_TOLERANCE)))
				{
					// Uncomment the below line if we can get good speed readings from the encoder
					if((myShooter->getRearWheelSpeed() <= (AUTONOMOUS_SPEED_TOLERANCE + AUTONOMOUS_REAR_WHEEL_SPEED) && myShooter->getRearWheelSpeed() >= (AUTONOMOUS_REAR_WHEEL_SPEED - AUTONOMOUS_SPEED_TOLERANCE))) {
					// Comment the below line if we get good encoders
					//if (autoSpinUpTimer->HasPeriodPassed(Config::GetSetting("auto_spinup_delay", 3.0)))
						if(myShooter->IsReady())
							autoState = 2;
					//else
						//myShooter->StopLoader();
					}
				}
				//else
				//{
				//	myShooter->StopLoader();
				//}
				break;
			case 2:
				lc->setMode(5); // firing lights :O
				myShooter->ChamberFrisbee();
				autoState = 1;
				break;
			case 3:
				myShooter->SetSpeeds(AUTONOMOUS_FRONT_WHEEL_SPEED,AUTONOMOUS_REAR_WHEEL_SPEED);
				myShooter->Process();
				if(myShooter->IsReady())
					autoState = 4;
				break;
			case 4:
				myShooter->SetSpeeds(AUTONOMOUS_FRONT_WHEEL_SPEED,AUTONOMOUS_REAR_WHEEL_SPEED);
				cout << "State 4"<<endl;
				//myShooter->ManualAngleControl(AUTO_ANGLE);
				myShooter->SetAngle(AUTO_ANGLE);
				myShooter->Process();
				if(myShooter->IsReady()) {
					autoState = 1;
					lc->setMode(4);
					autoSpinUpTimer->Start();
				}
				break;
			//case 5:
				//autoDelay = 
			default:
				cout << "NOT POSSIBLE" << endl;
			}
			
			break;
		
		default:
			cout<<"No such Autonomous Program"<<endl;
			break;
		}
		cout << "Auto state: " << autoState << endl;
		BuildAndSendPacket();
		logdata();
	}
    void TeleopPeriodic() {
    	//Target * target = GetLatestTarget();
    	//Begin DRIVER input section
    	if(driverGamepad->GetDPad()==Gamepad::kUp)
    		lc->setMode(1);
    	else if(driverGamepad->GetDPad()==Gamepad::kDown)
    		lc->setMode(6);
    	else
    		lc->setMode(0);
    	if(driverGamepad->GetDPad() == Gamepad::kRight)
    	{
    		cameraLight->Set(Relay::kOff);
    	}
    	else if(driverGamepad->GetDPad() == Gamepad::kLeft)
    	{
    		cameraLight->Set(Relay::kOn);
    	}
    	
    	if(driverGamepad->GetB())
    	{
    		cout << "Resetting the gyro! :D" << endl;
    		//driveGyro->Reset();
    		magicBox->resetGyro();
   		}
    	
    	bool modifySpeed = driverGamepad->GetRightBumper();
    	bool halfSpeed = driverGamepad->GetLeftBumper();
    	
    	/*
    	try {
    		std::string temp = SmartDashboard::GetString("TargetInfo");
    		cout << "temp: " << endl;
    	} catch (exception ex) {
    		cout << "bah! " << ex.what() << endl;
    	}*/
    	
    	//Target t = GetLatestTarget();
    	
    	/*
    	if (t.IsValid()) {
    		cout << "Valid target (" << (t.IsCenter() ? "center" : "side") << "):" << t.Distance() << " ft, " << t.X() << "," << t.Y() << endl;
    	} else {
    		//cout << "No valid target." << endl;
    	} */
    		
    	if(driverGamepad->GetBack())
    	{
    		cout << "Now driving in relation to the robot" << endl;
    			myDrive->setGyroDrive(false);
    	}
    	if(driverGamepad->GetStart())
    	{
    		cout << "Now driving in relation to the field (gyro)" << endl;
    		myDrive->setGyroDrive(true);
    	}
    	
    	// If the driver is holding the X button 
    	/*
    	if (driverGamepad->GetX()) {
    		// Stop using the joysticks to drive the robot, and let this function do it
    	    PointAtTarget(t);
    	} else { */
    		// We aren't auto-targeting. Reset the sequence.
    		target_state = RA13Robot::READY;
    		// Drive based on joysticks.
    		myDrive->Drive(driverGamepad->GetLeftX(), driverGamepad->GetLeftY(), 
    		    			driverGamepad->GetRightX(), magicBox->getGyroAngle(), modifySpeed,halfSpeed);
    	//}
    	//End DRIVER input section
    	
    	Gamepad::DPadDirection dir = operatorGamepad->GetDPad();
    	
    	if (dir != lastdir) {
    		switch(dir) {
    		case Gamepad::kUp:
    			manualAngle += 1;
    			break;
    		case Gamepad::kDown:
    			manualAngle -= 1;
    			break;
    		case Gamepad::kRight:
    			manualAngle += 0.25;
    			break;
    		case Gamepad::kLeft:
    			manualAngle -= 0.25;
    			break;
    		default:
    			break;
    		}
    	}
    	
    	//cout << "Current target angle: " << manualAngle << endl;
    	
    	//Begin OPERATOR input section
    		
    	if(operatorGamepad->GetBack())
    	{
    		myShooter->ResetShooterProcess(); // Re-homes the shooter
    	}
    	
    	if(operatorGamepad->GetLeftTrigger())
		{
    		myShooter->SetSpeeds(SHOOTER_FRONTWHEEL_SPEED,SHOOTER_REARWHEEL_SPEED); // (RPM,RPM)
		}
    	else if (operatorGamepad->GetLeftBumper())
    	{
    		myShooter->SetSpeeds(Config::GetSetting("shooter_main_speed_pyramid", 1500),
    							 Config::GetSetting("shooter_secondary_speed_pyramid", 2000));
    	}
    	else
    	{
    		myShooter->SetSpeeds(0,0); // (RPM,RPM)
    	}
    	
    	if(operatorGamepad->GetRightTrigger())
    	{
    		// TODO: Make this selectable somehow. This is probably too powerful for
    		// distant shots.
    		myShooter->ChamberFrisbee(Config::GetSetting("loader_power",-1));
    	}
    	else
    	{
    		myShooter->StopLoader();
    	}
    	
    	if(operatorGamepad->GetY())
    	{
    		myShooter->SetAngle(manualAngle);
    	}
    	if(operatorGamepad->GetX())
    	{
    		myShooter->SetAngle(Config::GetSetting("shooter_loading_angle",0)); //Angle for reloading
    		manualAngle = Config::GetSetting("shooter_loading_angle",0);
    	}
    	if(operatorGamepad->GetA()) //Shooting from load zone
    	{
    		myShooter->SetAngle(Config::GetSetting("load_zone_shooting_angle",13.25));
    		manualAngle = Config::GetSetting("load_zone_shooting_angle",13.25);
    	}
    	if(operatorGamepad->GetB())
    	{
    		myShooter->SetAngle(Config::GetSetting("pyramid_shooting_angle",23.25));
    		manualAngle = Config::GetSetting("pyramid_shooting_angle",23.25);
    	}
    	if(operatorGamepad->GetRightBumper())
    	{
    		//myShooter->SetAngle(Config::GetSetting("funnel_release_angle",35));
    		myShooter->SetAngle(Config::GetSetting("pyramid_basket_angle", 45));
    		manualAngle = Config::GetSetting("pyramid_basket_angle", 45);
    	}
    	if (operatorGamepad->GetLeftPush()) {
    		myShooter->ResetShooterProcess();
    	}
    	
    	// Do something similar, but don't touch the AngleJag.
    	if (operatorGamepad->GetRightPush()) {
    		myShooter->ResetShooterMotors();
    	}
    	
    	//pneumatics section
    	if(!driverGamepad->GetLeftTrigger())
    	{
    		pneumatics->extendEndgame();
    	}
    	else
    	{
    		pneumatics->retractEndgame();
    		myShooter->SetAngle(3);
    	}
    	
    	if(driverGamepad->GetRightTrigger())
    	{
    		pneumatics->extendCollector();
    	}
    	else
    	{
    		pneumatics->retractCollector();
    	}
    	//end pneumatics section
    	
    	lastdir = dir;
    	
    	myShooter->Process();
    	
    	//cout << "Shooter rear wheel" << myShooter->getRearWheelSpeed() << endl;
    	//End OPERATOR input section
   
    	
    	/*Manual control of the angle motor position using PID
    	 * THIS ASSUMES THE SHOOTER HAS BEEN ZERO'ED TO THE HOME POSITION SOMEHOW
    	if(operatorGamepad->GetLeftTrigger())
    	{
    		myShooter->ManualAngleControl(16); // (theta)
    	}
    	else
    	{
    		myShooter->ManualAngleControl(0);
    	}
    	*/ 
    	/*if (target != NULL) {
    		cout << "Target located. (" << target->X() << ',' << target->Y() << ") range is " << target->Distance() << " feet" << endl;
    	}
    	*/
//    	switch(myShooter->getFiringState())
//    	{
//    	case 1:
//    		lc->setMode(4);
////    		cout << "Spinning up!!" << endl;
//    		break;
//    	case 2:
//    		lc->setMode(5);
////    		cout << "Firing!!" << endl;
//    		break;
//    	default:
//    		lc->setMode(1);
//    		break;
//    	}
    	lc->useLights();
    	BuildAndSendPacket();
    	logdata();
    }
    
    void BuildAndSendPacket()
    {
    	Dashboard &dash = DriverStation::GetInstance()->GetLowPriorityDashboardPacker();
    	
    	dash.AddCluster();
    	{
    		dash.AddBoolean(myShooter->IsFrontWheelAtTargetSpeed());
    		dash.AddBoolean(myShooter->IsRearWheelAtTargetSpeed());
    		dash.AddFloat(myShooter->GetAngle());
    		dash.AddFloat(manualAngle);
    		
    		dash.AddFloat((float) myShooter->getFrontWheelTarget());
    		dash.AddFloat(myShooter->getFrontWheelSpeed());
    		dash.AddFloat((float) myShooter->getRearWheelTarget());
    		dash.AddFloat(myShooter->getRearWheelSpeed());
    	}
    	dash.FinalizeCluster();
    	dash.Finalize();
    }
    void TestPeriodic() 
    { 
    	myShooter->Process();
    	myShooter->SetAngle(0);
    	lc->useLights(); // ;)
    	cout << "Heading: " << magicBox->getGyroAngle() << endl;
    }

    /*
    Target GetLatestTarget() {
		//cout << "Getting target....";
		Target t = ts->GetLatestTarget();
		return t;
	} */
    /*
    void PointAtTarget(Target target)
    {
    	static float target_heading = 0;
    	try {
			float fov_width = Config::GetSetting("target_camera_fov", 57);
			float target_rotate_scale = Config::GetSetting("target_rotate_scale", .5);
			float rotation_tolerance = Config::GetSetting("target_rotate_tolerance", 1);
			float current_heading = magicBox->getGyroAngle();
			float rotation_output = 0;
			
			switch (target_state) {
			case RA13Robot::READY:
				// Want to target. 
				if (!target.IsValid()) {
					cout << "No target in sight. Ignoring." << endl;
					return;
				}
				
				// There's a target!
				
				float angle = target.X() * (fov_width / 2);
				target_heading = current_heading + angle;
				
				cout << "Angle = " << angle << endl;
				cout << "target heading = " << target_heading << endl;
				target_state = RA13Robot::MOVING;
				
				break;
			case RA13Robot::MOVING:
				if (::fabs(target_heading - current_heading) < rotation_tolerance) {
					target_state = RA13Robot::DONE;
				} else {
					float err = target_heading - current_heading;
					cout << "err = " << err << endl;
					// Normalize: treat error as if it was between -1..1
					// and multiply it by target_rotate_scale
					rotation_output = (err / (fov_width / 2)) * target_rotate_scale;
				}
				
				break;
			case RA13Robot::DONE:
				cout << "Yo, you're already pointing at the target, man!" << endl;
				break;
			}
			
			myDrive->Drive(0,0,rotation_output, current_heading, true,false);
    	} catch (exception ex) {
    		cout << "Exception: " << ex.what() << endl;
    	}
    }*/
    
    /**
     * @brief Retrieves the checksum of the current executable
     * 
     * Retrieves and displays the checksum of the binary as a 
     * guard against accidental upload of temporary programs.
     */
    void GetChecksum() {
    	FILE * binary = ::fopen("/c/ni-rt/system/FRC_UserProgram.out", "rb");
    	size_t bytes_read = 0;
    	uint16_t check = Fletcher16(binary, &bytes_read );
    	fclose(binary);
    	
    	if (binary == NULL) {
    		std::cerr << "Not able to open FRC_UserProgram.out for input" << std::endl ;
    	}
    	std::cout << "Code checksum: 0x" << std::ios_base::fixed <<  std::setprecision(8) << std::hex << check << std::dec << std::endl;
    	std::cout << "Binary length:   " << bytes_read << " bytes" << std::endl;
    }
    //logging functions
    void logheaders()
    {
    	fout << "Time,Mode,Battery,Gyro,";
    	autoLogHeaders();
    	myDrive->logHeaders(fout);
    	myShooter->logHeaders(fout);
    	//myClimber->LogHeaders(fout);
    	log_gamepad_headers(fout, "Driver");
    	log_gamepad_headers(fout, "Gamepad");
    	//autoLogHeaders();
    	fout << "FancyLights,";
    	fout << "Light1,";
    	fout << "Light2,";
    	fout<<endl;
    }
    
    void log_gamepad_headers(ostream & out, std::string name)
    {
    	char * fields[15] = {
    			"A",
    			"B",
    			"X",
    			"Y",
    			"DPad",
    			"Left X",
    			"Left Y",
    			"Right X",
    			"Right Y",
    			"Left Trigger",
    			"Right Trigger",
    			"Left Bumper",
    			"Right Bumper",
    			"Start",
    			"Back",
    	};
    	
    	for (int i = 0; i < 15; ++i) {
    		out << name << ' ' << fields[i] << ',';
    	}
    }
    
    void log_gamepad(ostream & out,Gamepad * gp)
    {
    	out << gp->GetA() << ',';
    	out << gp->GetB() << ',';
    	out << gp->GetX() << ',';
    	out << gp->GetY() << ',';
    	out << gp->GetDPad() << ',';
    	out << gp->GetLeftX() << ',';
    	out << gp->GetLeftY() << ',';
    	out << gp->GetRightX() << ',';
    	out << gp->GetRightY() << ',';
    	out << gp->GetLeftTrigger() << ',';
    	out << gp->GetRightTrigger() << ',';
    	out << gp->GetLeftBumper() << ',';
    	out << gp->GetRightBumper() << ',';
    	out << gp->GetStart() << ',';
    	out << gp->GetBack() << ',';
    }
    void logdata()
    {
    	//log data
    	fout << logTimer->Get() << ',' << CurrentMode() << ',' << ds->GetBatteryVoltage() << ',';
    	fout << magicBox->getGyroAngle() << ',';
    	autoLog();
    	myDrive->log(fout);
    	myShooter->log(fout);
    	//myClimber->Log(fout);
    	log_gamepad(fout, driverGamepad);
    	log_gamepad(fout, operatorGamepad);
    	fout<<LEDTimer<<",";
    	fout<<LEDPercent(LEDTimer)<<",";
    	fout<<LEDPercent(LEDTimer + 30)<<",";
    	fout<<endl;
    }
    void autoLogHeaders()
    {
    	fout<<"FrontShooterSetPoint,FrontShooterActual,RearShooterSetPoint,RearShooterActual,AutoState,";
    }
    void autoLog()
    {
    	fout<<AUTONOMOUS_FRONT_WHEEL_SPEED << ',' << myShooter->getFrontWheelSpeed() << ',';
    	fout << AUTONOMOUS_REAR_WHEEL_SPEED<<',' << myShooter->getRearWheelSpeed() << ',';
    	fout << autoState << ',';
    }
    void ResetShooterMotors()
    {
    	/*
    	FrontShooterMotor->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		FrontShooterMotor->ConfigEncoderCodesPerRev(100);
		FrontShooterMotor->SetPID(Config::GetSetting("shooter_kp", -0.25),
								  Config::GetSetting("shooter_ki", -0.005),
								  Config::GetSetting("shooter_kd", -0.0001));
		FrontShooterMotor->EnableControl(0);
		
		RearShooterMotor->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		RearShooterMotor->ConfigEncoderCodesPerRev(100);
		RearShooterMotor->SetPID(Config::GetSetting("rear_kp", -0.25),
								 Config::GetSetting("rear_ki", -0.005),
								 Config::GetSetting("rear_kd", -0.0001));
		RearShooterMotor->EnableControl(0);
		*/
    }
    
    void OutputTroll(ostream & out) {
    	out << 
"                                      ....... .\n"                            
"                      ,7IB@MMMMMMMMMMMMMMMMMMM@@@MMMMMMMMMM,               \n"
"                ,I#MMMMMMM$ZI7Yii,,.,....     ..         :#MM              \n"
"             YMMMWoci,.      ..,:iii,,.........,,:ii.       MMo            \n"
"           EMM9       ,,:,...:;;ci,    ...::;;YYY;;icY;      2M#           \n"
"          MM;     .;:,.,,ii;;         Ui     ...,vSUv;iY;.    AMz          \n"
"         ,M      SI        Yi        Ai            .v;;,..     @M          \n"
"         MM      .          S            ,C6SotXi.     .        MM         \n"
"        MM.        ioAU1;   ,         YMMMbiMMMMMMM@i            MM;       \n"
"      2MM  ,..,; :MMMMMMMMM@.       .MM,   ;MMMMMM#@MM iSY    .i,iMMMv     \n"
"     MMv.YI.      ,iYt68B@MMMMMb    cMMt@MM$Y.C.  iobC   .,CZEb;   .bMM.   \n"
"    MM.,:Y..YoY,            Mo        S$:     CM#.     nMMM@1zbMMQ    nMt  \n"
"    M@, , vMM#E@MMX.@2      M.                 .$MMMMMMMZ.  @    MM  . 7M, \n"
"    MMi Y     I iMMMM,    MMM         vi                 .UMM$    MM i  M@ \n"
"    SM9  C   :M.       iMMb          :@MMMvi:.,.      9MMMZ oMMMn MM ,  MM \n"
"     MMC.7,  MMC    .cCMMMz      YMMMM, #M      .7$MMMMo    MM i;tM. 7 ;M7 \n"
"      MM .Y MM@M$.        MM; A. .;. . .M   7WMMMM$,M$    7MMY   X, C iM#  \n"
"       M@   MMMCMMM@i      .#MW      .vE$MMM#A:     MM1MMMMM7       ,MMo   \n"
"       zM   MME M1 SMMMMMM@o8@$@MMMMMM#Ii.Mt     ;MMMMM: MM        0MM     \n"
"        MY .MM6 Mi  MC    cM8    M@       ZMi$MMMMMQM;  MM        MM,      \n"
"        Mz  MMMMMM6EMz.   ,M.    M#  .;z@MMMMMM#7   MoYMM        MM        \n"
"        Mz  MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMCM@       tMMS        oM        \n" 
"        M;  EMAMMMMMMMMMMMMMMMMMMMMMZ7i    ZM     8MM8         XMC     \n"    
"       .M.   Mb9M MM :M@ .YMM     .M        @M ;MMMz          MM7   \n"       
"       SM    ,MMM  M7 ,M    M     .M       ,$MMMb       i.  @MM  \n"          
"       MM      6MMMMM$nMMc. MM.:::IMMW#MMMMMW,   .:;c1SYioMMM.\n"             
"       M@          .c6WE#MMMMMMMMMMQ$Q8C:     iYCYvi,,X#MM9\n"                
"       M2  S.  i8,                       .;2Uo;,..,2@MMZ\n"                   
"       Mc  .X,   v1Xv:.           ,,vXoUASv.   v$MM@Y\n"                      
"       M;    7Iz;............,.,,:::..     .8MMM2.\n"                         
"       MM       .,:::........           ;@MM$:\n"                             
"        MMz                     .z@MMMMMMb.\n"                                
"         iMMME7.       .,i1Z#MMMMMMES;.\n"                                    
"            C@MMMMMMMMMMMM@Wn;.\n"                                            
"\n";
    }
    void callMe()
    {
    	cout<<"Hey I just met you."<<endl;
    	cout<<"And this is crazy."<<endl;
    	cout<<"So here's my number,"<<endl;
    	cout<<"So call me, maybe?"<<endl;
    }
    double LEDPercent(double x)
    {
    	double xDegrees = 359 * (x / 60);
    	double xRadians = xDegrees * (3.14159 / 180);
    	return 0.5*cos(xRadians) + 0.5;
    }
};

START_ROBOT_CLASS(RA13Robot);
