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


// =============================================================================
//  Record-and-Replay Robotic Arm (VEX IQ)
//
//  Controls:
//    Joysticks        - manual control (base, shoulder, gripper)
//    LDown (hold)     - record a movement
//    LUp              - start replay / press again to stop
//    RUp or Touch LED - shut down (arm returns to home position)
//
//  Touch LED colours:
//    Blue = manual, blinking red = recording, green = replaying,
//    purple = shutting down
// =============================================================================


// -----------------------------------------------------------------------------
// Devices and global variables
// -----------------------------------------------------------------------------
motor baseMotor = motor(PORT1, false);
motor v1Motor = motor(PORT2, false);
motor gripperMotor = motor (PORT4, false);
bool gripperPickup[1000] = {};      // true if an object was in the gripper at each recorded sample
optical Optical3 = optical(PORT5);
distance Distance = distance(PORT6);
touchled TouchLED9 = touchled(PORT9);
bool running = true;
timer timeStop;

// Joint limits in degrees, so the joints cannot over-rotate and pull on the wires
const double BaseLimit = 270.0;
const double v1Limit = 180.0;
const double gripLimit = 360.0;


// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void InitProgram()
{
  BrainInertial.calibrate();
  while (BrainInertial.isCalibrating()) {}
  BrainInertial.resetRotation();
  Brain.Screen.setCursor(1, 1);
}


// -----------------------------------------------------------------------------
// Manual control
// -----------------------------------------------------------------------------

// Converts a joystick reading (-100..100) into a motor speed (-maxSpeed..maxSpeed).
// Small readings are ignored (deadband) to filter out controller drift.
int JoystickToSpeed(int axisValue, int maxSpeed)
{
   // Readings under 5 are treated as controller drift
   if(abs(axisValue) < 5)
   {
        return 0;
   }

   // Speed is proportional to the joystick position,
   // e.g. 50% on the joystick with a max speed of 100 gives a speed of 50
   return axisValue * maxSpeed / 100;
}

// Runs one joint at a signed speed (positive = forward, negative = reverse),
// but only while it stays between 0 and its joint limit.
// The motor is passed by reference so the real motor object is controlled.
void driveJoint(motor &joint, int speed, int jointLimit)
{
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
        joint.stop();   // at a limit or stick released: uses the stopping mode set in main
    }
}


// -----------------------------------------------------------------------------
// Recording and replay settings
// -----------------------------------------------------------------------------
double BaseMotorPos[1000] = {};     // recorded base positions (up to 1000 samples)
double v1MotorPos[1000] = {};       // recorded shoulder positions
double gripperMotorPos[1000] = {};  // recorded gripper positions
int SampleCount = 0;                // how many samples have been recorded so far
timer SampleTimer;                  // stopwatch used while recording
const int SAMPLE_MS = 50;           // record one sample every 50 ms
const int MAX_SAMPLES = 1000;       // the arrays are full after 1000 samples (50 seconds)
const double TOLERANCE = 3;         // degrees counted as "close enough" to a target
const double HOME_POS = 30;         // the position all joints return to on shutdown

// Replay tuning (change these while testing)
const double SNAP = 1.5;            // speed (%) per degree of error. Higher = snappier, too high = shaking
const double MIN_SPEED = 8;         // slowest speed used, so the motor actually moves
const double MAX_SPEED = 70;        // fastest speed allowed during replay
const double DEADBAND = 2;          // within this many degrees the motor just holds


// -----------------------------------------------------------------------------
// Recording
// -----------------------------------------------------------------------------

// Returns true if an object is within 60 mm of the gripper's distance sensor
bool IsObjectPickup()
{
    if(Distance.objectDistance(mm) < 60){
        return true;
    }
    else{
        return false;
    }
}

// Starts a new recording (overwrites the old one)
void StartRecording()
{
    SampleCount = 0;        // reset the sample count
    SampleTimer.clear();    // reset the recording timer
}

// Called every pass of the main loop while recording.
// Every 50 ms, saves each joint's position and whether an object is held.
void Record()
{
    if (SampleCount < MAX_SAMPLES && SampleTimer.time(msec) >= SampleCount * SAMPLE_MS) // checks if 50 ms has passed
    {
        BaseMotorPos[SampleCount] = baseMotor.position(degrees);
        v1MotorPos[SampleCount] = v1Motor.position(degrees);
        gripperMotorPos[SampleCount] = gripperMotor.position(degrees);
        gripperPickup[SampleCount] = IsObjectPickup();

        SampleCount++;
    }
}


// -----------------------------------------------------------------------------
// Replay
// -----------------------------------------------------------------------------

// Smoothly pushes one joint toward Target (proportional control):
// far from the target = fast, close to the target = slow.
void FollowPos (motor &joint, double Target)
{
    double error = Target - joint.position(degrees); // difference between target and actual position

    if (fabs(error) <= DEADBAND){   // close enough: hold position
        joint.stop();
        return;
    }

    double speed = fabs(error) * SNAP;
    if (speed < MIN_SPEED){
        speed = MIN_SPEED;          // too slow won't move the motor, so raise it
    }
    if (speed > MAX_SPEED){
        speed = MAX_SPEED;          // too fast is unsafe, so cap it
    }

    if (error > 0)  // target is above the current position
    {
        joint.spin(forward, speed, percent);
    }
    else            // target is below the current position
    {
        joint.spin(reverse, speed, percent);
    }
}

// True when the joint is within TOLERANCE (3 degrees) of the desired position
bool Arrived(motor &joint, double DesiredPos)
{
    return fabs(joint.position(degrees) - DesiredPos) <= TOLERANCE;
}

// True when the user presses LUp again (or RUp, the exit button)
bool StopRequested()
{
    return Controller.ButtonLUp.pressing() || Controller.ButtonRUp.pressing();
}

// Press LUp once to start replay, press LUp again (or RUp) to stop it.
// All joints move together and follow the recording by time, blending between
// samples, so the motion is smooth and runs at the same speed it was recorded.
// Replay loops until stopped, or until the gripper loses the object.
void Replay()
{
    if (SampleCount == 0){  // nothing recorded yet
        return;
    }

    // Wait until LUp is released. LUp is also the stop button, so without
    // this the same press that started replay would instantly stop it.
    while (Controller.ButtonLUp.pressing()) { wait(10, msec); }

    timer ReplayTimer;

    bool pickupFailed = false;
    timer MissTimer;
    bool missing = false;   // true while the recording and the sensor disagree

    // Outer loop: repeat the whole replay until stop is pressed
    while (!StopRequested() && !pickupFailed)
    {
        // PART 1: move to the first recorded pose (give up after 3 s)
        ReplayTimer.clear();
        while (!StopRequested() && ReplayTimer.time(msec) < 3000 &&
               !(Arrived(baseMotor, BaseMotorPos[0]) && Arrived(v1Motor, v1MotorPos[0]) && Arrived(gripperMotor, gripperMotorPos[0])))
        {
            FollowPos(baseMotor, BaseMotorPos[0]);
            FollowPos(v1Motor, v1MotorPos[0]);
            FollowPos(gripperMotor, gripperMotorPos[0]);
            wait(10, msec);
        }

        // PART 2: play the recording back by time.
        // Samples are 50 ms apart but this loop runs every 10 ms, so each target
        // is blended between sample i and sample j = i + 1 (linear interpolation):
        //   target = start + (end - start) * fraction
        double totalTime = (SampleCount - 1) * SAMPLE_MS;
        ReplayTimer.clear();
        while (!StopRequested() && !pickupFailed && ReplayTimer.time(msec) <= totalTime)
        {
            double exact = ReplayTimer.time(msec) / (double)SAMPLE_MS;   // e.g. 3.4 = 40% of the way from sample 3 to 4
            int i = (int)exact;             // current sample (e.g. 3)
            double fraction = exact - i;    // how far toward the next sample (e.g. 0.4)
            int j = i + 1;                  // next sample (e.g. 4)
            if (j >= SampleCount)
            {
                j = i;   // don't go past the end of the recording
            }

            FollowPos(baseMotor, BaseMotorPos[i] + (BaseMotorPos[j] - BaseMotorPos[i]) * fraction);
            FollowPos(v1Motor, v1MotorPos[i] + (v1MotorPos[j] - v1MotorPos[i]) * fraction);
            FollowPos(gripperMotor, gripperMotorPos[i] + (gripperMotorPos[j] - gripperMotorPos[i]) * fraction);

            // Pickup check: the recording was holding an object here, but the sensor sees nothing
            if(gripperPickup[i] && !IsObjectPickup())
            {
                if(!missing)
                {
                    MissTimer.clear();   // the mismatch just started
                    missing = true;
                }
                else if(MissTimer.time(msec) > 500)
                {   // object missing for over 0.5 s: stop the replay
                    pickupFailed = true;
                }
            }
            else
            {
                missing = false; // object detected again, reset
            }

            wait(10, msec); // loop runs every 10 ms
        }

        wait(10, msec);
    }

    while (Controller.ButtonLUp.pressing()) { wait(10, msec); }   // so the stop press doesn't restart replay

    baseMotor.stop();
    v1Motor.stop();
    gripperMotor.stop();
}


// -----------------------------------------------------------------------------
// Shutdown
// -----------------------------------------------------------------------------

// Moves one joint toward defaultPos at a slow, fixed speed
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

// Moves every joint toward HOME_POS
void ShutDown ()
{
    DefaultPos(baseMotor, HOME_POS);
    DefaultPos(v1Motor, HOME_POS);
    DefaultPos(gripperMotor, HOME_POS);
}


// -----------------------------------------------------------------------------
// Controls
// -----------------------------------------------------------------------------

// Reads the joysticks and drives each joint:
//   Axis C = base, Axis D = shoulder, Axis B = gripper
// RUp ends the program.
void UpdateControls()
{
    driveJoint(baseMotor, JoystickToSpeed(Controller.AxisC.position(), 100), BaseLimit);
    driveJoint(v1Motor, JoystickToSpeed(Controller.AxisD.position(), 70) , v1Limit);
    driveJoint(gripperMotor, JoystickToSpeed(Controller.AxisB.position(), 70) , gripLimit);

    if (Controller.ButtonRUp.pressing())
    {
      running = false;
    }
}


// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main()
{
    vexcodeInit();
    Brain.Screen.printAt(10, 20,  "Final Project");
    Brain.Screen.printAt(10, 40,  "Manual: RDown");
    Brain.Screen.printAt(10, 60,  "Record:  LDown");
    Brain.Screen.printAt(10, 80,  "Replay:  LUp");
    Brain.Screen.printAt(10, 100, "Shutdown: RUp");
    InitProgram();

    baseMotor.setStopping(hold);
    v1Motor.setStopping(hold);
    gripperMotor.setStopping(brake); // brake grips objects better when not moving

    bool wasRecording = false;

    TouchLED9.on(blue);

    while(running)
    {
        UpdateControls(); // check every input for changes
        wait(10,msec);    // short delay between inputs so control is smoother

        // Hold LDown to record. Releasing it ends the recording.
        if (Controller.ButtonLDown.pressing())
        {
            if (!wasRecording)   // only on the first loop of the press
            {
                StartRecording();
                Brain.Screen.clearScreen();
                Brain.Screen.printAt ( 10 , 40, "Recording..." );
                TouchLED9.setBlink(red, 0.5, 0.5);
            }
            wasRecording = true;

            Record();   // every loop while LDown is held
        }
        else
        {
            if (wasRecording)   // recording just ended, back to manual
            {
                TouchLED9.on(blue);
                Brain.Screen.clearScreen();
                Brain.Screen.printAt(10, 40, "Manual Mode");
            }

            wasRecording = false;
        }

        // LUp: pause manual control and replay the recording
        if (Controller.ButtonLUp.pressing())
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

    // Shutdown: return every joint to HOME_POS, or give up after 10 seconds
    const int SHUTDOWN_TIMEOUT_MS = 10000;
    timeStop.clear();   // restart the timer right before the loop

    TouchLED9.on(purple);
    Brain.Screen.clearScreen();
    Brain.Screen.printAt ( 4 , 20, "Shutting down..." );

    // keep going while ANY joint is away from HOME_POS and we haven't timed out
    while ((fabs(baseMotor.position(degrees) - HOME_POS) > TOLERANCE ||
            fabs(v1Motor.position(degrees) - HOME_POS) > TOLERANCE ||
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
    gripperMotor.stop();
    wait(2, sec);
    Brain.programStop();

    return EXIT_SUCCESS;
}
