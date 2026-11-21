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

#include "iq_cpp.h"

using namespace vex;


// ---------------------------------------------------------------------------
// Devices
// ---------------------------------------------------------------------------
motor baseMotor    = motor(PORT1, false);
motor v1Motor      = motor(PORT2, false);
motor gripperMotor = motor(PORT4, false);
optical Optical3   = optical(PORT5);
distance Distance  = distance(PORT6);
touchled TouchLED9 = touchled(PORT9);

bool running = true;
timer timeStop;


// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

// Joint limits (degrees) to protect the wiring during manual control
const double BaseLimit = 270.0;
const double v1Limit   = 180.0;
const double gripLimit = 360.0;

// Recording
const int    SAMPLE_MS   = 50;      // one sample every 50 ms
const int    MAX_SAMPLES = 1000;    // 1000 samples = 50 s of recording
const double TOLERANCE   = 3;       // degrees counted as "arrived"
const double HOME_POS    = 30;      // joint position on shutdown

// Replay (proportional control)
const double SNAP      = 1.5;       // speed (%) per degree of error
const double MIN_SPEED = 8;         // lowest speed that still moves the motor
const double MAX_SPEED = 70;        // highest speed allowed during replay
const double DEADBAND  = 2;         // within this many degrees the motor holds

// Gripper object detection
const double PICKUP_DISTANCE_MM = 60;
const int    PICKUP_FAIL_MS     = 500;


// ---------------------------------------------------------------------------
// Recorded data
// ---------------------------------------------------------------------------
double BaseMotorPos[MAX_SAMPLES]    = {};
double v1MotorPos[MAX_SAMPLES]      = {};
double gripperMotorPos[MAX_SAMPLES] = {};
bool   gripperPickup[MAX_SAMPLES]   = {};
int    SampleCount = 0;
timer  SampleTimer;


// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void InitProgram()
{
    BrainInertial.calibrate();
    while (BrainInertial.isCalibrating()) {}
    BrainInertial.resetRotation();
    Brain.Screen.setCursor(1, 1);
}


// ---------------------------------------------------------------------------
// Manual control
// ---------------------------------------------------------------------------

// Converts a joystick reading (-100..100) into a motor speed (-maxSpeed..maxSpeed).
// Small readings are treated as stick drift and ignored.
int JoystickToSpeed(int axisValue, int maxSpeed)
{
    if (abs(axisValue) < 5)
    {
        return 0;
    }
    return axisValue * maxSpeed / 100;
}

// Spins one joint at a signed speed, stopping at 0 and at the joint's limit.
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
        joint.stop();
    }
}

void UpdateControls()
{
    driveJoint(baseMotor,    JoystickToSpeed(Controller.AxisC.position(), 100), BaseLimit);
    driveJoint(v1Motor,      JoystickToSpeed(Controller.AxisD.position(), 70),  v1Limit);
    driveJoint(gripperMotor, JoystickToSpeed(Controller.AxisB.position(), 70),  gripLimit);

    if (Controller.ButtonRUp.pressing())
    {
        running = false;
    }
}


// ---------------------------------------------------------------------------
// Recording
// ---------------------------------------------------------------------------
bool IsObjectPickup()
{
    return Distance.objectDistance(mm) < PICKUP_DISTANCE_MM;
}

void StartRecording()
{
    SampleCount = 0;
    SampleTimer.clear();
}

// Called every main-loop pass while recording; saves one sample every SAMPLE_MS.
void Record()
{
    if (SampleCount < MAX_SAMPLES && SampleTimer.time(msec) >= SampleCount * SAMPLE_MS)
    {
        BaseMotorPos[SampleCount]    = baseMotor.position(degrees);
        v1MotorPos[SampleCount]      = v1Motor.position(degrees);
        gripperMotorPos[SampleCount] = gripperMotor.position(degrees);
        gripperPickup[SampleCount]   = IsObjectPickup();

        SampleCount++;
    }
}


// ---------------------------------------------------------------------------
// Replay
// ---------------------------------------------------------------------------

// Moves one joint toward Target: faster when far away, slower when close.
void FollowPos(motor &joint, double Target)
{
    double error = Target - joint.position(degrees);

    if (fabs(error) <= DEADBAND)
    {
        joint.stop();
        return;
    }

    double speed = fabs(error) * SNAP;
    if (speed < MIN_SPEED)
    {
        speed = MIN_SPEED;
    }
    if (speed > MAX_SPEED)
    {
        speed = MAX_SPEED;
    }

    if (error > 0)
    {
        joint.spin(forward, speed, percent);
    }
    else
    {
        joint.spin(reverse, speed, percent);
    }
}

bool Arrived(motor &joint, double DesiredPos)
{
    return fabs(joint.position(degrees) - DesiredPos) <= TOLERANCE;
}

bool StopRequested()
{
    return Controller.ButtonLUp.pressing() || Controller.ButtonRUp.pressing();
}

// Plays the recording back by time, blending between samples every 10 ms so the
// motion is smooth and runs at the recorded speed. Repeats until stopped.
void Replay()
{
    if (SampleCount == 0)
    {
        return;
    }

    // LUp also stops replay, so wait for the starting press to be released.
    while (Controller.ButtonLUp.pressing()) { wait(10, msec); }

    timer ReplayTimer;
    timer MissTimer;
    bool pickupFailed = false;
    bool missing = false;

    while (!StopRequested() && !pickupFailed)
    {
        // Part 1: move to the first recorded pose (up to 3 s)
        ReplayTimer.clear();
        while (!StopRequested() && ReplayTimer.time(msec) < 3000 &&
               !(Arrived(baseMotor, BaseMotorPos[0]) &&
                 Arrived(v1Motor, v1MotorPos[0]) &&
                 Arrived(gripperMotor, gripperMotorPos[0])))
        {
            FollowPos(baseMotor, BaseMotorPos[0]);
            FollowPos(v1Motor, v1MotorPos[0]);
            FollowPos(gripperMotor, gripperMotorPos[0]);
            wait(10, msec);
        }

        // Part 2: follow the recording by time
        double totalTime = (SampleCount - 1) * SAMPLE_MS;
        ReplayTimer.clear();
        while (!StopRequested() && !pickupFailed && ReplayTimer.time(msec) <= totalTime)
        {
            double exact = ReplayTimer.time(msec) / (double)SAMPLE_MS;
            int i = (int)exact;
            double fraction = exact - i;
            int j = i + 1;
            if (j >= SampleCount)
            {
                j = i;
            }

            FollowPos(baseMotor,    BaseMotorPos[i]    + (BaseMotorPos[j]    - BaseMotorPos[i])    * fraction);
            FollowPos(v1Motor,      v1MotorPos[i]      + (v1MotorPos[j]      - v1MotorPos[i])      * fraction);
            FollowPos(gripperMotor, gripperMotorPos[i] + (gripperMotorPos[j] - gripperMotorPos[i]) * fraction);

            // Stop if the gripper should be holding an object but has lost it
            if (gripperPickup[i] && !IsObjectPickup())
            {
                if (!missing)
                {
                    MissTimer.clear();
                    missing = true;
                }
                else if (MissTimer.time(msec) > PICKUP_FAIL_MS)
                {
                    pickupFailed = true;
                }
            }
            else
            {
                missing = false;
            }

            wait(10, msec);
        }

        wait(10, msec);
    }

    // Wait for release so the stop press doesn't restart replay
    while (Controller.ButtonLUp.pressing()) { wait(10, msec); }

    baseMotor.stop();
    v1Motor.stop();
    gripperMotor.stop();
}


// ---------------------------------------------------------------------------
// Shutdown
// ---------------------------------------------------------------------------
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

void ShutDown()
{
    DefaultPos(baseMotor, HOME_POS);
    DefaultPos(v1Motor, HOME_POS);
    DefaultPos(gripperMotor, HOME_POS);
}


// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
    vexcodeInit();
    Brain.Screen.printAt(10, 20,  "Final Project");
    Brain.Screen.printAt(10, 40,  "Manual: Joysticks");
    Brain.Screen.printAt(10, 60,  "Record:  LDown");
    Brain.Screen.printAt(10, 80,  "Replay:  LUp");
    Brain.Screen.printAt(10, 100, "Shutdown: RUp");

    InitProgram();

    baseMotor.setStopping(hold);
    v1Motor.setStopping(hold);
    gripperMotor.setStopping(brake);

    bool wasRecording = false;
    TouchLED9.on(blue);

    while (running)
    {
        UpdateControls();
        wait(10, msec);

        // Hold LDown to record. wasRecording catches the first loop of each press.
        if (Controller.ButtonLDown.pressing())
        {
            if (!wasRecording)
            {
                StartRecording();
                Brain.Screen.clearScreen();
                Brain.Screen.printAt(10, 40, "Recording...");
                TouchLED9.setBlink(red, 0.5, 0.5);
            }
            wasRecording = true;
            Record();
        }
        else
        {
            if (wasRecording)
            {
                TouchLED9.on(blue);
                Brain.Screen.clearScreen();
                Brain.Screen.printAt(10, 40, "Manual Mode");
            }
            wasRecording = false;
        }

        // Press LUp to replay. Replay() runs its own loop until stopped.
        if (Controller.ButtonLUp.pressing())
        {
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(10, 40, "Replaying...");
            TouchLED9.on(green);
            Replay();

            TouchLED9.on(blue);
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(10, 40, "Manual Mode");
        }

        if (TouchLED9.pressing())
        {
            running = false;
        }
    }

    // Return every joint to HOME_POS, or give up after the timeout
    const int SHUTDOWN_TIMEOUT_MS = 10000;
    timeStop.clear();

    TouchLED9.on(purple);
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 40, "Shutting down...");

    while ((fabs(baseMotor.position(degrees)    - HOME_POS) > TOLERANCE ||
            fabs(v1Motor.position(degrees)      - HOME_POS) > TOLERANCE ||
            fabs(gripperMotor.position(degrees) - HOME_POS) > TOLERANCE)
           && timeStop.time(msec) < SHUTDOWN_TIMEOUT_MS)
    {
        ShutDown();
        wait(10, msec);
    }

    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 40, "Shutdown Complete");

    TouchLED9.off();
    baseMotor.stop();
    v1Motor.stop();
    gripperMotor.stop();
    wait(2, sec);
    Brain.programStop();

    return EXIT_SUCCESS;
}
