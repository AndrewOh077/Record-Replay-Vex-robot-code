#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>








#include "vex.h"




using namespace vex;




// Brain should be defined by default
brain Brain;








// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))




#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS




// Robot configuration code.
inertial BrainInertial = inertial();
controller Controller = controller();






// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed);
}






void vexcodeInit() {




  // Initializing random seed.
  initializeRandomSeed();
}






// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;




#pragma endregion VEXcode Generated Robot Configuration




// Include the IQ Library
#include "iq_cpp.h"






// Allows for easier use of the VEX Library
using namespace vex;






// global varibles for all functions
// delcaring motors and sensros
motor baseMotor = motor(PORT1, false);
motor v1Motor = motor(PORT2, false);
motor v2Motor = motor (PORT3, false);
motor gripperMotor = motor (PORT4, false);
bool gripperPickup[1000] = {};      // gripper object pickup array
optical Optical3 = optical(PORT5);
distance Distance = distance(PORT6);
touchled TouchLED9 = touchled(PORT9);
bool running = true;
timer timeStop;






// Joint rotation limits; validate against the physical arm and wiring.
const double BaseLimit = 270.0;
const double v1Limit = 225.0;
const double v2Limit = 135.0;
const double gripLimit = 360.0;






void InitProgram()
{
  BrainInertial.calibrate();
  while (BrainInertial.isCalibrating()) {}
  gripperMotor.spin(reverse, 100, percent);
  wait(1, seconds);
  gripperMotor.stop();
  Brain.Screen.setCursor(1, 1);
  BrainInertial.resetRotation();
  //Brain.Screen.printAt( 2, 30, "Final Project" );
  // INSERT MOTOR CALIBRATION FOR POSITION (GO ALL THE WAY LEFT, THEN CERTAIN AMOUNT RIGHT TO DEFAULT POSITION)
  // RESET ENCODES TO ZERO TO TARE
  // Wait for orders
}








// Converts a stick reading (-100..100) into a motor speed (-maxSpeed..maxSpeed).
// Ignores small readings (deadband) and uses a squared curve so a light push
// gives fine, slow control and a full push gives full speed.
int JoystickToSpeed(int axisValue, int maxSpeed)
{
   // axisValue is the position given from the controller, max speed is 0-100
   if(abs(axisValue) < 5)
   { // if its less than 5, it is insignificant  and probably controller drift
        return 0;
   }


   // axisValue from the position mutiplied by maxspeed/100 returns
   // the speed relative to the controller position
   // ie 50% on the joystick with a max speed of 100 is speed = 50
   return axisValue * maxSpeed / 100;
}


int V1JoystickToSpeed(int axisValue, int maxSpeed)
{
   // axisValue is the position given from the controller, max speed is 0-100
   if(abs(axisValue) < 5)
   { // if its less than 5, it is insignificant  and probably controller drift
        return 0;
   }
   if (axisValue < 0)
   {
    return (axisValue * (maxSpeed-20) / 100);
   }


   // axisValue from the position mutiplied by maxspeed/100 returns
   // the speed relative to the controller position
   // ie 50% on the joystick with a max speed of 100 is speed = 50
   return axisValue * maxSpeed / 100;
}






// Runs one joint at a signed speed (positive = forward, negative = reverse).
// The motor is passed by reference so the real motor object is controlled.
void driveJoint(motor &joint, int speed, int jointLimit)
{
    //bool if_limit_true = false;
    //check for limits and see if it exceeds or not, if it exceeds, it stays false
    double pos = joint.position(degrees);




    if (speed > 0 && pos < jointLimit)
    {
        joint.spin(forward, speed, percent);
    }
    else if (speed < 0 && pos > 0)
    {
        joint.spin(reverse, -speed, percent);
    }
    else
    {
        joint.stop();   // at a limit or stick released: uses the stopping mode from main
    }
}






double BaseMotorPos[1000] = {};     // list of 1000 numbers for the base motor, all start at 0 (ASSUMING NOT LONGER THAN 1000 inputs)
double v1MotorPos[1000] = {};       // same for motor v1 (shoulder)
double v2MotorPos[1000] = {};       // same for motor v2 (elbow)
double gripperMotorPos[1000] = {};  // same for the gripper
int SampleCount = 0;                // how many samples have been recorded so far
timer SampleTimer;                  // a stopwatch used while recording
const int SAMPLE_MS = 50;           // record one sample every 50 ms
const int MAX_SAMPLES = 1000;       // the arrays are full after 1000 samples (50 seconds)
const double TOLERANCE = 3;         // "close enough" to a target cause we know that it aint perfect
const double HOME_POS = 0;         // the position all joints return to on shutdown




// Replay tuning (change these while testing)
const double SNAP = 1.5;              // speed (%) per degree of error. Higher = snappier, too high = shaking
const double MIN_SPEED = 8;         // slowest speed used, so the motor actually moves
const double MAX_SPEED = 70;        // fastest speed allowed during replay
const double DEADBAND = 2;          // within this many degrees the motor just holds


// retruns bool value if obj is in gripper grasp
bool IsObjectPickup()
{
    if(Distance.objectDistance(mm) < 20){ // if object is within distance, return true
        return true;
    }
    else{
        return false;
    }
}






// starts/resets old recording
void StartRecording()
{
    SampleCount = 0;        // resets to zero
    SampleTimer.clear();    // resets timer for recording
}




// calls this every pass of the main loop, during recording it:
// sets poition every 50ms
// puts this position in a point within the array for each motor indiviually
void Record()
{  
    if (SampleCount < MAX_SAMPLES && SampleTimer.time(msec) >= SampleCount * SAMPLE_MS) // checks if 50ms has passed
    {
        // record !!! YAYYAYAY
        BaseMotorPos[SampleCount] = baseMotor.position(degrees);
        v1MotorPos[SampleCount] = v1Motor.position(degrees);
        v2MotorPos[SampleCount] = v2Motor.position(degrees);
        gripperMotorPos[SampleCount] = gripperMotor.position(degrees);
        gripperPickup[SampleCount] = IsObjectPickup();




        SampleCount++;
    }
}


// smoothly pushes one joint toward Target (proportional control)
void FollowPos (motor &joint, double Target)
{
    double error = Target - joint.position(degrees); // error is the difference from target/reality


    if (fabs(error) <= DEADBAND){ // CHECKS IF ERROR IS WITHIN CLOSE ENOUGH RANGE
        joint.stop();           // Hold position when within tolerance.
        return;
    }


    double speed = fabs(error) * SNAP;      // far away = fast, close = slow
    if (speed < MIN_SPEED){
        speed = MIN_SPEED;         // too slow won't move the motor, so raise it
    }
    if (speed > MAX_SPEED){
        speed = MAX_SPEED;         // too fast is unsafe, so cap it
    }


    if (error > 0)  // target is above the current position
    {
        joint.spin(forward, speed, percent);
    }
    else  // target is below the current position
    {
        joint.spin(reverse, speed, percent);
    }
}




// EXTRA FUNCTIONS TO HELP OUT MAIN REPLAY FUNCTION




bool Arrived(motor &joint, double DesiredPos) // true when within tolerance
{
    return fabs(joint.position(degrees) - DesiredPos) <= TOLERANCE;  // true if within 3 degrees
}




// true when the user presses LUp again (or RUp, the exit button)
bool StopRequested()
{
    return Controller.ButtonLUp.pressing() || Controller.ButtonRUp.pressing(); // true if either button is held
}


// Press LUp once to start Replay, press LUp again (or RUp) to stop it.
// All 4 joints move together, following the recording by TIME (with in-between values),
// so the motion is smooth and has the same speed as when you recorded it.
void Replay()
{
    if (SampleCount == 0){ // IF THE SAMPLE HAD 0, THEN THATS AN MISINPUT/ERROR
        return; // No recording available.
    }


    // wait until the finger is off LUp. LUp is also a stop button, so without
    // this the same press that started replay would instantly stop it.
    while (Controller.ButtonLUp.pressing()) { wait(10, msec); }   // wait until the start press is released


    timer ReplayTimer; // a stopwatch for the replay


    bool pickupFailed = false;
    timer MissTimer;
    bool missing = false;   // true while recording and sensor disagree




    while (!StopRequested() && !pickupFailed) // outer loop: repeat the whole replay until stop is pressed
    // outer loop: repeat the whole replay until stop is pressed
    {
        // PART 1: move to the first recorded pose (give up after 3 s)


        ReplayTimer.clear(); // reset the stopwatch
        // keeps looping until the following:
        // no stop requested, less than 3 seconds per entry, and all motors arent t base position
        while (!StopRequested() && ReplayTimer.time(msec) < 3000 &&
               !(Arrived(baseMotor, BaseMotorPos[0]) && Arrived(v1Motor, v1MotorPos[0]) && Arrived(v2Motor, v2MotorPos[0]) && Arrived(gripperMotor, gripperMotorPos[0])))
        {
            // BEFORE REPLAYS, RESETS MOTORS TO THE FIRST POSITION IN THE ENTRY
            FollowPos(baseMotor, BaseMotorPos[0]);
            FollowPos(v1Motor, v1MotorPos[0]);
            FollowPos(v2Motor, v2MotorPos[0]);
            FollowPos(gripperMotor, gripperMotorPos[0]);
            wait(10, msec);
        }




        // PART 2: play the recording back by time
        double totalTime = (SampleCount - 1) * SAMPLE_MS;
        ReplayTimer.clear();
        while (!StopRequested() && !pickupFailed && ReplayTimer.time(msec) <= totalTime)
        {
            // SINCE THE RECORDING IS EVERY 0.05s AND MAIN CODE RUNS AT A 'FRAME RATE'
            // OF 0.01s
            // Interpolate between recorded motor positions
            // e.g. DIVDE TIME AND MOVEMENT TO CALC MOVEMENT EVERY 0.01S TO REACH
            // POINT B IN 0.05s
           
            // example, POINT I = 30, POINT J = 40
            double exact = ReplayTimer.time(msec) / (double)SAMPLE_MS;   // e.g. 3.4 = 40% between sample 3 and 4
            int i = (int)exact; //time on replay timer / 50ms : Z   (i=3)
            double fraction = exact - i; //decimal part of i
            int j = i + 1; // this would be the next entry after I (j=4)
            if (j >= SampleCount)
            {
                j = i;   // don't go past the end of the list
            }


            // Follow(MOTOR, START + (END - START)*FRACTION )
            // i = first entry
            // j = next entry
            // (END - START)*FRACTION ---- the difference is how much we need to go, *fraction is the decimal (50ms/10ms)
            FollowPos(baseMotor, BaseMotorPos[i] + (BaseMotorPos[j] - BaseMotorPos[i]) * fraction);
            FollowPos(v1Motor, v1MotorPos[i] + (v1MotorPos[j] - v1MotorPos[i]) * fraction);
            FollowPos(v2Motor, v2MotorPos[i] + (v2MotorPos[j] - v2MotorPos[i]) * fraction);
            FollowPos(gripperMotor, gripperMotorPos[i] + (gripperMotorPos[j] - gripperMotorPos[i]) * fraction);


            if(gripperPickup[i] && !IsObjectPickup())
            { // recorded entry is true, real is false
                if(!missing)
                {
                    MissTimer.clear();   // the mismatch just started
                    missing = true;
                }
                else if(MissTimer.time(msec) > 500)
                {   // 500 ms grace period for a missed pickup
                    pickupFailed = true;
                    // PRINT TO BRAIN THAT IT FAILED !!
                }
            }
            else
            {
                missing = false; // resets after it detects object
            }


            wait(10, msec); // framerate bascailly
        }


        wait(10, msec);
    }


    while (Controller.ButtonLUp.pressing()) { wait(10, msec); }   // so the stop press doesn't restart Replay


    baseMotor.stop();
    v1Motor.stop();
    v2Motor.stop();
    gripperMotor.stop();
}








void DefaultPos(motor &joint, double defaultPos)
{
    directionType direction;


    if (joint.position(degrees) < defaultPos)
    {
        direction = forward;
    }
    else
    {
        direction = reverse;
    }


    if (fabs(joint.position(degrees) - defaultPos) > TOLERANCE)
    {
        joint.spin(direction, 25, percent);
    }
    else
    {
        joint.stop();
    }
}




void ShutDown ()
{
    DefaultPos(baseMotor, HOME_POS);
    DefaultPos(v1Motor, HOME_POS);
    DefaultPos(v2Motor, HOME_POS);
    DefaultPos(gripperMotor, HOME_POS);
}

const int ACCEL_STEP = 10;   // % speed change per loop. Smaller = smoother but slower to respond

int Ramp(int current, int target)
{
    if (target > current + ACCEL_STEP)
        return current + ACCEL_STEP;     // speeding up (or turning)
    if (target < current - ACCEL_STEP)
        return current - ACCEL_STEP;     // slowing down (or reversing)
    return target;                       // close enough, jump to it
}


int baseSpeed = 0;
int v1Speed   = 0;
int v2Speed   = 0;
int gripSpeed = 0;

void UpdateControls()
{
    // BASE MOTOR UPDATING CONTROL: C-AXIS IS LEFT/RIGHT FOR R3
    // CALLS driveJoint which takes the motor and speed
    // speed is determined by joystickspeed whcih does a calc based on joystick
    // driveJoint spins the motor
    // same thing for all the other motors:
    baseSpeed = Ramp(baseSpeed, JoystickToSpeed(-Controller.AxisC.position(), 100));
    v1Speed   = Ramp(v1Speed,   V1JoystickToSpeed(Controller.AxisD.position(), 40));
    v2Speed   = Ramp(v2Speed,   JoystickToSpeed(-Controller.AxisA.position(), 70));
    gripSpeed = Ramp(gripSpeed, JoystickToSpeed(-Controller.AxisB.position(), 70));

    driveJoint(baseMotor, baseSpeed, BaseLimit);
    driveJoint(v1Motor, v1Speed , v1Limit);
    driveJoint(v2Motor, v2Speed , v2Limit);
    driveJoint(gripperMotor, gripSpeed , gripLimit);


    if (Controller.ButtonRUp.pressing())
    {
      running = false;
    }
}








int main()
{
    vexcodeInit();
    Brain.Screen.printAt(10, 20,  "Final Project");
    Brain.Screen.printAt(10, 40,  "Manual: Joysticks");
    Brain.Screen.printAt(10, 60,  "Record:  LDown");
    Brain.Screen.printAt(10, 80,  "Replay:  LUp");
    Brain.Screen.printAt(10, 100, "Shutdown: RUp");
    // Print these when the we have Replay, delete the others. Need to write that in the actual function
    InitProgram();


    baseMotor.setStopping(hold);
    v1Motor.setStopping(hold);
    v2Motor.setStopping(hold);
    gripperMotor.setStopping(brake); // better so it hold objects with more grip when not moving




    bool wasRecording = false;  


    TouchLED9.on(blue);




    baseMotor.setMaxTorque(100,percent);
    v1Motor.setMaxTorque(100,percent);
    v2Motor.setMaxTorque(100,percent);
    gripperMotor.setMaxTorque(100,percent);
    while(running)
    {
        UpdateControls(); // checks every input to see any changes
        wait(10,msec); // delay between inputs so its smoother


        // Recording and replay controls
        // while Ldown is holding, record all movements during that time and store it in an array which updates in size depending on time
        if (Controller.ButtonLDown.pressing())
        {
            if (!wasRecording)
            {
                StartRecording();
                Brain.Screen.clearScreen();
                Brain.Screen.printAt ( 10 , 40, "Recording..." );
                TouchLED9.setBlink(red, 0.5, 0.5);  
                // only on the first loop of the press
            }
            wasRecording = true;


            Record();               // every loop while LDown is held
        }
        else
        {
            if (wasRecording)                      // recording just ended → back to manual
            {
                TouchLED9.on(blue);
                Brain.Screen.clearScreen();
                Brain.Screen.printAt(10, 40, "Manual Mode");
            }


            wasRecording = false;
        }




        if (Controller.ButtonLUp.pressing()) // STOPS MAIN CONTROL UPDATING AND REPLACES IT WITH
        {
            Brain.Screen.clearScreen();
            Brain.Screen.printAt ( 10 , 40, "Replaying..." );
            TouchLED9.on(green);
            Replay();


            TouchLED9.on(blue);
            Brain.Screen.clearScreen();
            Brain.Screen.printAt ( 10 , 40, "Manual Mode" );
        }


        if (TouchLED9.pressing())
        {
            running = false;
        }
    }


    const int SHUTDOWN_TIMEOUT_MS = 10000;
    timeStop.clear();   // restart the timer right before the loop


    TouchLED9.on(purple);
    Brain.Screen.clearScreen();
    Brain.Screen.printAt ( 4 , 20, "Shutting down..." );


    // keep going while ANY joint is away from HOME_POS AND we haven't timed out
   while ((fabs(baseMotor.position(degrees) - HOME_POS) > TOLERANCE ||
        fabs(v1Motor.position(degrees)      - HOME_POS) > TOLERANCE ||
        fabs(v2Motor.position(degrees)      - HOME_POS) > TOLERANCE ||
        fabs(gripperMotor.position(degrees) - HOME_POS) > TOLERANCE)
        && timeStop.time(msec) < SHUTDOWN_TIMEOUT_MS)
    {
        ShutDown();
        wait(10, msec);
    }


    Brain.Screen.clearScreen();
    Brain.Screen.printAt ( 4 , 20, "Shutdown Complete" );


    TouchLED9.off();
    baseMotor.stop();
    v1Motor.stop();
    v2Motor.stop();
    gripperMotor.stop();
    wait(1, sec);
    Brain.programStop();




    return EXIT_SUCCESS;
}




// setMaxTorque();


