//Note: Code was from before solenoid codes were fixed at SCIRW

#include "WPILib.h"
#include "NetworkTables/NetworkTable.h"
#include "RobotMap.h"
 
class TM_2014_ROBOT : public IterativeRobot {
    // Controllers
    Joystick *controller;
    Joystick *joystick;
 
    // Robot drive system
    RobotDrive *tmRobotDrive;
 
    // Talon motors
    Talon *frontLeftWheel;
    Talon *rearLeftWheel;
    Talon *frontRightWheel;
    Talon *rearRightWheel;
     
    // Limit Switch
    DigitalInput* lockingLS;
     
    // Compressor and solonoids
    Compressor* compressor;
    DoubleSolenoid* solenoidLaunch;
    DoubleSolenoid* solenoidBlocker;
    DoubleSolenoid* solenoidLock;
     
    // Robot statuses
    enum eLauncherStatus {LAUNCHER_READY, LAUNCHER_PRESSURIZING, LAUNCHER_LOCKED, LAUNCHER_DOWN, LAUNCHER_RAISED, ABNORMAL_STATE};
    eLauncherStatus launcherStatus;
     
    // Digital Outputs (spike relays)
    Relay* cameraLight;
     
    // Gyro
    Gyro* gyro;
     
    // Timers
    Timer* timerLaunch;
    Timer* timerAuto;
     
    DriverStationLCD *dsLCD;
    NetworkTable *coordinatesTable;
     
    float speed;
    float strafe;
    float rotation;
     
    float armSpeed;
    float ballGatherSpeed;
    float launchDelay;
 
    int autonomousState;
public:
    // Initialize robot variables here
    TM_2014_ROBOT(void) {
        // Initialize the  joystick
        controller = new Joystick(CONTROLLER);
        joystick   = new Joystick(JOYSTICK);
         
        // Initialize Talons
        frontLeftWheel   = new Talon(FRONT_LEFT_WHEEL);
        rearLeftWheel    = new Talon(REAR_LEFT_WHEEL);
        frontRightWheel  = new Talon(FRONT_RIGHT_WHEEL);
        rearRightWheel   = new Talon(REAR_RIGHT_WHEEL);
         
        // Initialize Limit Switch
        lockingLS = new DigitalInput(LOCKING_MECHANISM_LS);
         
        // Initialize compressor and solonoids
        compressor      = new Compressor(COMPRESSOR_IN, COMPRESSOR_OUT);
        solenoidLaunch  = new DoubleSolenoid(GREEN_FILL,GREEN_DROP);
        solenoidLock    = new DoubleSolenoid(RED_LOCK,RED_LAUNCH);
        solenoidBlocker = new DoubleSolenoid(BLOCKER_DOWN,BLOCKER_UP);
         
        // Initialize Digital Outputs
        cameraLight = new Relay(CAMERA_LED);
         
        // Initialize gyro
        gyro = new Gyro(GYRO);
 
        // Initialize timers
        timerLaunch = new Timer();
        timerAuto = new Timer();
         
        dsLCD = DriverStationLCD::GetInstance();
        coordinatesTable = NetworkTable::GetTable("Target Status Table");
 
        // Initialize robot drive system with the two wheels
        tmRobotDrive = new RobotDrive(frontLeftWheel,rearLeftWheel,frontRightWheel,rearRightWheel);
         
        // Invert left wheel motors
        tmRobotDrive->SetInvertedMotor(tmRobotDrive->kFrontRightMotor,true);
        tmRobotDrive->SetInvertedMotor(tmRobotDrive->kRearRightMotor,true);
         
        armSpeed = 0.8;
        ballGatherSpeed = 0.6;
        launchDelay = 10.0;
    }
 
    /*************************** Miscellaneous Commands ***********************/   
    bool isActive(DigitalInput* limitSwitch) {
        return limitSwitch->Get();
    }
    float thresholdValue(float value, float thresh = 0.2) {
        //IS-3 is best tonk
        if (value > 0.0) if (value < thresh) return 0.0;
        else if (value < 0.0) if (value > -thresh) return 0.0;
        return value;
    }
    void toggleLED(string state) {
        if (state == "on") cameraLight->Set(Relay::kForward);
        else if (state == "off") cameraLight->Set(Relay::kOff);
    }
    void indexLauncherStatus() {
        // if loader down, locked, and fully pressurized
//      if (isActive(lockingLS) && launcherLocked() && launcherPressurized() && timerLaunch->Get() > launchDelay)
        if (launcherLocked() && launcherPressurized() && timerLaunch->Get() > launchDelay)
            launcherStatus = LAUNCHER_READY;
        // if loader down, locked, and pressurizing
//      else if (isActive(lockingLS) && launcherLocked() && launcherPressurized() && timerLaunch->Get() < launchDelay)
        else if (launcherLocked() && launcherPressurized() && timerLaunch->Get() < launchDelay)
            launcherStatus = LAUNCHER_PRESSURIZING;
        // if loader down, locked and not pressurizing
//      else if (isActive(lockingLS) && launcherLocked() && launcherDown())
        else if (launcherLocked() && launcherDown())
            launcherStatus = LAUNCHER_LOCKED;
        // if loader down, not locked and not pressurizing
//      else if (isActive(lockingLS) && launcherReleased() && launcherDown())
        else if (launcherReleased() && launcherDown())
            launcherStatus = LAUNCHER_DOWN;
        // if loader not down and not locked
//      else if (!isActive(lockingLS) && launcherReleased())
        else if (launcherReleased())
            launcherStatus = LAUNCHER_RAISED;
        // if none of the above are true (e.g. launcher raised and locked)
        else
            launcherStatus = ABNORMAL_STATE;
    }
    void printMessage(char* message, char lineNum) {
        dsLCD->PrintfLine((DriverStationLCD::Line) lineNum, message);
        dsLCD->UpdateLCD();
    }
    void displayStatusOnDashboard(char lineNum = 1) {
        if (launcherStatus == LAUNCHER_READY)
            printMessage("Launcher Ready", lineNum);
        else if (launcherStatus == LAUNCHER_PRESSURIZING)
            printMessage("Launcher Pressurizing", lineNum);
        else if (launcherStatus == LAUNCHER_LOCKED)
            printMessage("Launcher Locked", lineNum);
        else if (launcherStatus == LAUNCHER_DOWN)
            printMessage("Launcher Dropped", lineNum);
        else if (launcherStatus == LAUNCHER_RAISED)
            printMessage("Launcher Raised", lineNum);
        else if (launcherStatus == ABNORMAL_STATE)
            printMessage("Abnormal State", lineNum);
    }
    bool getXboxButton(int btnNum) {
        return controller->GetRawButton(btnNum);
    }
    bool getJoystickButton(int btnNum) {
        return joystick->GetRawButton(btnNum);
    }
    float getXboxAxis(int axisNum) {
        return controller->GetRawAxis(axisNum);
    }
    float getJoystickAxis(int axisNum) {
        return joystick->GetRawAxis(axisNum);
    }
    /******************************* Drive Commands ****************************/
    void mecDrive(float x, float y, float rotation, float gyroAngle = 0.0) {
        tmRobotDrive->MecanumDrive_Cartesian(x, y, rotation, gyroAngle);
    }
    void stopRobot() {
        mecDrive(0.0,0.0,0.0);
    }
    void teleopDrive() {
        speed = -thresholdValue(getXboxAxis(LEFT_ANALOG_Y),0.3);
        strafe = -getXboxAxis(TRIGGERS);
        rotation = thresholdValue(controller->GetRawAxis(RIGHT_ANALOG_X),0.3);
        mecDrive(strafe,speed,rotation);
    }
    void teleopShooter() {
        if (joystick->GetTrigger())
            launchBall();
        if (getJoystickButton(3))
            moveBlockerUp();
        if (getJoystickButton(2))
            moveBlockerDown();
        if (getJoystickButton(10))
            freeSolenoids();
        if (getJoystickButton(11))
            pressurizeLauncher();
        if (getJoystickButton(6))
            extend(solenoidLock);
        if (getJoystickButton(7))
            retract(solenoidLock);
    }
    /******************************* Pneumatics Commands ***********************/
    void retract(DoubleSolenoid* solenoid) {
        solenoid->Set(DoubleSolenoid::kReverse);
    }
    void extend(DoubleSolenoid* solenoid) {
        solenoid->Set(DoubleSolenoid::kForward);
    }
    bool isRetracted(DoubleSolenoid* solenoid) {
        return solenoid->Get() == DoubleSolenoid::kReverse;
    }
    bool isExtended(DoubleSolenoid* solenoid) {
        return solenoid->Get() == DoubleSolenoid::kForward;
    }
    void pressurizeLauncher() {
        indexLauncherStatus();
        if (launcherStatus == LAUNCHER_LOCKED)
            retract(solenoidLaunch);
        displayStatusOnDashboard();
    }
    void dropLauncher() {
        indexLauncherStatus();
        if (launcherStatus == LAUNCHER_RAISED)
            extend(solenoidLaunch);
        displayStatusOnDashboard();
    }
    void lockLauncher() {
        indexLauncherStatus();
        if (launcherStatus == LAUNCHER_DOWN)
            extend(solenoidLock);
        displayStatusOnDashboard();
    }
    void releaseLauncher() {
        indexLauncherStatus();
        if (launcherStatus == LAUNCHER_READY)
            retract(solenoidLock);
        displayStatusOnDashboard(); 
    }
    void initializeSolenoids(bool autonomous = true) {
        if (autonomous) {
            extend(solenoidLaunch);
            Wait(0.5);
        }
        extend(solenoidLock);
        Wait(1.0);
        retract(solenoidLaunch);
        displayStatusOnDashboard();
    }
    void freeSolenoids() {
        extend(solenoidLaunch);
        displayStatusOnDashboard();
    }
    bool launcherPressurized() {
        return isRetracted(solenoidLaunch);
    }
    bool launcherDown() {
        return isExtended(solenoidLaunch);
    }
    bool launcherLocked() {
        return isExtended(solenoidLock);
    }
    bool launcherReleased() {
        return isRetracted(solenoidLock);
    }
    /******************************* Launcher Commands *************************/
    void launchBall(bool indexStatus = true) {
        if (indexStatus)
            indexLauncherStatus();
        displayStatusOnDashboard();
        if (launcherStatus == LAUNCHER_READY) {
            releaseLauncher();
            Wait(1.0);
            displayStatusOnDashboard();
            dropLauncher();
//          while (launcherStatus != LAUNCHER_DOWN) {
//              indexLauncherStatus();
//              displayStatusOnDashboard();
//          }
            Wait(2.0);
            launcherStatus = LAUNCHER_DOWN;
            displayStatusOnDashboard();
            lockLauncher();
            pressurizeLauncher();
            displayStatusOnDashboard();
            timerLaunch->Reset();
        }
    }
    void moveBlockerUp() {
        retract(solenoidBlocker);
    }
    void moveBlockerDown() {
        extend(solenoidBlocker);
    }
    /********************************* Test Commands ***************************/
    void testLimitSwitch(char lineNum = 1) {
        if (lockingLS->Get() == false)
            printMessage("Locking false", lineNum);
        else if (lockingLS->Get() == true)
            printMessage("Locking true", lineNum);  
    }
    void lightControl() {
        if (controller->GetRawAxis(DPAD_X) < 0.0)
            toggleLED("off");
        if (controller->GetRawAxis(DPAD_X) > 0.0)
            toggleLED("on");
    }   
 
    void testSolenoids(char lineNum = 2) {
        if (getXboxButton(A)) {
            extend(solenoidLaunch);
            printMessage("Launch extended", lineNum);
        }
        else if (getXboxButton(Y)) {
            retract(solenoidLaunch);
            printMessage("Launch retracted", lineNum);
        }
        else if (getXboxButton(RIGHT_BUMPER)) {
            retract(solenoidLock);
            printMessage("Lock retracted", lineNum);
        }
        else if (getXboxButton(LEFT_BUMPER)) {
            extend(solenoidLock);
            printMessage("Lock extended", lineNum);
        }
        else if (getXboxButton(X)) {
            retract(solenoidBlocker);
            printMessage("Blocker retracted", lineNum);
        }
        else if (getXboxButton(B)) {
            extend(solenoidBlocker);
            printMessage("Blocker extended", lineNum);
        }
        else if (getXboxButton(LEFT_ANALOG_PRESS)) {
            initializeSolenoids();
            printMessage("Solenoids reset", lineNum);
        }
        indexLauncherStatus();
        displayStatusOnDashboard();
    }
    /****************************** Networking Commands *************************/
    void placeTargetStatus(string targetStatus) {
        coordinatesTable->PutString("Target Status", targetStatus);
    }
    string getTargetStatus() {
        return coordinatesTable->GetString("Target Status");
    }
    bool targetDetected() {
        return getTargetStatus() == "Target Detected";
    }
    void printTargetStatus(char lineNum = 2) {
        if (targetDetected())
            printMessage("Target Detected", lineNum);
        else
            printMessage("No Target Detected", lineNum);
    }
    /********************************** Init Routines *****************************************/
    void RobotInit(void) {
        dsLCD->Clear();
        printMessage("Robot Enabled", 0);
        gyro->Reset();
        compressor->Start();
    }
    void DisabledInit(void) {
        dsLCD->Clear();
        printMessage("Robot Disabled", 0);
        gyro->Reset();
    }
    void AutonomousInit(void) {
        dsLCD->Clear();
        printMessage("Autonomous Mode", 0);
        gyro->Reset();
        timerLaunch->Start();
        timerLaunch->Reset();
        timerAuto->Start();
        timerAuto->Reset();
        autonomousState = 0;
        moveBlockerDown();
        initializeSolenoids();
        //pressurizeLauncher();
        indexLauncherStatus();
        displayStatusOnDashboard();
        //moveBlockerDown();
        //placeTargetStatus("No Target Detected");
    }
    void TeleopInit(void) {
        dsLCD->Clear();
        printMessage("Teleop Mode", 0);
        gyro->Reset();
        timerLaunch->Start();
        timerLaunch->Reset();
        //pressurizeLauncher();
        indexLauncherStatus();
        initializeSolenoids(false);
        indexLauncherStatus();
        displayStatusOnDashboard();
        //placeTargetStatus("No Target Detected");
    }
    /********************************** Periodic Routines *************************************/
    void DisabledPeriodic(void) {
        // no code
    }
    void AutonomousPeriodic(void) {
        dsLCD->Clear();
        printMessage("Autonomous Enabled", 0);
        indexLauncherStatus();
        //if (launcherStatus == ABNORMAL_STATE)
            //initializeSolenoids();
        //if (timerLaunch->Get() < launchDelay && launcherStatus == LAUNCHER_DOWN)
        //  lockLauncher();
        displayStatusOnDashboard();
        switch(autonomousState) {
        case 0:
            toggleLED("on");
            //pressurizeLauncher();
            //if (launcherStatus == LAUNCHER_READY)
            //  autonomousState = 1;
            //timerLaunch->Reset();
            timerAuto->Reset();
            autonomousState = 1;
            break;
        case 1:
            //launchBall();
            //if (launcherStatus == LAUNCHER_PRESSURIZING)
            //  autonomousState = 2;
            while (timerAuto->Get() < 1.2)
                mecDrive(0.0,1.0,0.0);
            //timerAuto->Reset();
            //while (timerAuto->Get() < 0.7)
            //  mecDrive(0.0,-1.0,0.0);
            //timerAuto->Reset();
            //while (timerAuto->Get() < 0.7)
            //  mecDrive(0.0,1.0,0.0);
            stopRobot();
            autonomousState = 2;
            break;
        case 2:
            toggleLED("off");
            //launcherStatus = LAUNCHER_READY;
            //launchBall(false);
            autonomousState = 3;
            break;
        default:
            break;
        }
    }
    void TeleopPeriodic(void) {
        dsLCD->Clear();
        printMessage("Teleop Enabled", 0);
        teleopDrive();
        indexLauncherStatus();
        displayStatusOnDashboard();
        teleopShooter();
        //testSolenoids();
        //printTargetStatus();
    }
};
     
START_ROBOT_CLASS(TM_2014_ROBOT);