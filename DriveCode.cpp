#include "WPILib.h" // This imports the WPI Library, which includes (almost) everything we need to program the robot

/* This is the skeleton of the IterativeRobot program. It is basically a series of functions
 * that the dashboard runs at specified times. Init functions only run once each time it's called,
 * and Periodic functions loop infinitely while the robot is in its specified mode (Autonomous,
 * Teleop, etc.).
 */

class TM_2013_Robot : public IterativeRobot
{
	// Declare robot variables here
	Talon* frontLeftWheel;
	Talon* frontRightWheel;
	Talon* backLeftWheel;
	Talon* backRightWheel;

	Joystick* controller;
	RobotDrive* tmRobotDrive;

	Timer* timer;
	DriverStationLCD *dsLCD;

	Gyro* gyro;

	static const int A				 	= 1;
	static const int B 				 	= 2;
	static const int X 				 	= 3;
	static const int Y 				 	= 4;
	static const int LEFT_BUMPER  	    = 5;
	static const int RIGHT_BUMPER 		= 6;
	static const int BACK 			    = 7;
	static const int HOME			    = 8;
	static const int LEFT_ANALOG_PRESS  = 9;
	static const int RIGHT_ANALOG_PRESS = 10;

public:
	// Initialize robot variables here
	TM_2013_Robot(void) {
		frontLeftWheel = new Talon(1);
		frontRightWheel = new Talon(2);
		backLeftWheel = new Talon(3);
		backRightWheel = new Talon(4);

		controller = new Joystick(1);

		gyro = new Gyro(1);

		tmRobotDrive = new RobotDrive(frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel);

		/// invert all the motors
		tmRobotDrive->SetInvertedMotor(tmRobotDrive->kFrontRightMotor,true);
		tmRobotDrive->SetInvertedMotor(tmRobotDrive->kRearRightMotor,true);

		timer = new Timer();
		dsLCD = DriverStationLCD::GetInstance();
	}

	/********************************** Command Functions *************************************/
	// Define command functions here
	void tankDrive(float leftSpeed, float rightSpeed) {
		tmRobotDrive->TankDrive(leftSpeed,rightSpeed);
	}
	void mecDrive(float x, float y, float rotation) {
		tmRobotDrive->MecanumDrive_Cartesian(x, y, rotation);
	}
	void stopRobot() {
		tankDrive(0.0,0.0);
	}
	void printMessage(char* message, char lineNum) {
		dsLCD->PrintfLine((DriverStationLCD::Line) lineNum, message);
		dsLCD->UpdateLCD();
	}
	void driveStraight(float time, float speed) {
		while (timer->Get() < time) {
			float angle = gyro->GetAngle();
			leftSpeed  = speed-(angle/50.0);
			rightSpeed = speed+(angle/50.0);
			// make sure motor is in range [-1.0,1.0]
			if (leftSpeed < 0.0) {
				leftSpeed = max(leftSpeed,-1.0);
			}
			if (leftSpeed > 0.0) {
				leftSpeed = min(leftSpeed,1.0);
			}
			if (rightSpeed < 0.0) {
				rightSpeed = max(rightSpeed,-1.0);
			}
			if (leftSpeed > 0.0) {
				rightSpeed = min(rightSpeed,1.0);
			}
			tankDrive(leftSpeed, rightSpeed);
		}
		stopRobot();
	}
	void turnRight(float angle, float rotationSpeed) {
		gyro->Reset();
		while(gyro->GetAngle() < angle){
			mecDrive(0.0, 0.0, rotationSpeed);
		}
		stopRobot();
	}

	//driveStraight(10.0);

	/********************************** Init Routines *****************************************/
	// Runs once when the robot is turned on
	void RobotInit(void) {

	}
	// Runs once when the robot is disabled
	void DisabledInit(void) {

	}
	// Runs once when autonomous mode is initialized
	void AutonomousInit(void) {
		timer->Reset();
		timer->Start();
		gyro->Reset();
	}
	// Runs once when teleop mode is initialized
	void TeleopInit(void) {
		gyro->Reset();
	}

	/********************************** Periodic Routines *************************************/
	// Runs while the robot is disabled
	void DisabledPeriodic(void) {

	}
	// Runs while the robot is in autonomous mode (after being initialized)
	void AutonomousPeriodic(void) {
		printMessage("HI I am in autonimous mode", 0); // print this messsage on the first line
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Time: %f", timer->Get()); // print the elapsed time
		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Angle: %f", gyro->GetAngle());
		if (timer->Get() < 2.0) { // for two seconds...
			mecDrive(0.0,0.5,0.0); // drive forwards at half speed
		}
		else {
			Wait(1.0); // pause the code for one second
			timer->Reset(); // reset the timer
		}
		dsLCD->UpdateLCD();
	}
	// Runs while the robot is teleop mode (after being initialized)
	void TeleopPeriodic(void) {
		float speed = -controller->GetRawAxis(2);
		float rotation = controller->GetRawAxis(4);
		float strafe = controller->GetRawAxis(3);
		mecDrive(strafe, speed, rotation);
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 2, "Angle: %f", gyro->GetAngle());
		if(controller->GetRawButton(A)){
			printMessage("Button A works",5);
			gyro->Reset();
		}
		dsLCD->UpdateLCD();
	}
};

START_ROBOT_CLASS(TM_2013_Robot);
