# Record-and-Replay VEX IQ Robotic Arm

A **four-motor VEX IQ robot arm** controller supporting manual joystick operation, joint-position recording, and timed motion replay.

> **Status:** Four-motor source code is available on this update branch, adapted from the latest team project file. The earlier three-motor implementation remains on `main`. The four-motor revision has **not been compiled or retested on the physical robot as part of this GitHub update**. Hardware-specific settings and pickup detection should be verified before treating it as a fully tested build.

## Features

- **Manual control:** Joysticks control the base, shoulder, elbow, and gripper; speed ramping helps smooth acceleration.
- **Record:** Saves all four encoder positions at **50 ms** intervals (up to 1,000 samples / 50 seconds), along with object-presence readings.
- **Replay:** Interpolates between samples at approximately **10 ms** update intervals and uses proportional motor-position feedback to follow the recorded trajectory.
- **Pickup check:** Compares live distance-sensor readings against recorded pickup states during replay, with a **500 ms** mismatch grace period; a failed pickup stops replay. Sensor threshold and connectivity must be tested on the robot.
- **Shutdown:** Attempts to return all four motors to the home position with a 10-second timeout, then stops the motors.
- **Mode indicators:** Brain screen messages and Touch LED for manual, recording, replay, and shutdown states.

## Motor and sensor configuration in code

| Device | Port | Role |
| --- | --- | --- |
| Base motor | 1 | Base rotation |
| Shoulder motor (`v1Motor`) | 2 | Shoulder joint |
| Elbow motor (`v2Motor`) | 3 | Elbow joint |
| Gripper motor | 4 | Gripper |
| Optical sensor | 5 | Declared; no active behavior here |
| Distance sensor | 6 | Recorded pickup state / replay pickup check |
| Touch LED | 9 | Mode feedback / shutdown input |

Joystick assignments in `UpdateControls()`: **Axis C** base, **Axis D** shoulder, **Axis A** elbow, **Axis B** gripper. **LDown** holds recording, **LUp** starts/stops replay, and **RUp/Touch LED** requests shutdown.

## Technical implementation

- Four arrays store individual motor encoder positions, sampled every 50 ms.
- Linear interpolation sets intermediate targets between recorded positions.
- `FollowPos()` uses proportional control with speed caps and a small position deadband.
- `Ramp()` limits changes in commanded manual joystick speed.
- The elbow motor is included in movement, recording, playback, starting-pose verification, and shutdown.
- Software angle limits, zero/home positions, distance thresholds, and motor directions remain **hardware-dependent tuning values**, not validated safe physical limits.

## Before hardware testing

1. Verify motor ports, controller axes, motor orientation, travel limits, cable clearances, and home positions with **no load**.
2. Check the distance sensor is connected and reporting valid readings; test actual object-present/object-absent distances before relying on the current **20 mm** threshold.
3. Confirm the gripper's initialization motion can run safely.
4. Test recording, interpolation, missed-pickup stopping, and each of the four shutdown movements at low speed.
5. Retune limits and drive response for any chain-driven gear reduction. The encoder measures the motor shaft, not the output joint.
6. Compile and validate using the actual VEX IQ development environment; no hardware or compilation verification was performed in this update.

## Source provenance

The four-motor implementation is based on the team's later project code shared in the ongoing development conversation. It supersedes the earlier three-motor source for this **proposed** update but does not imply a physical validation result.
