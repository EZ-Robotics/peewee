# peewee

My robot for the SPUR-FLYS minigame, [Mini-Up](https://docs.google.com/document/d/1mCmWGhqI1ZUdg05hKQo3zhatCqkk73Puq9dKWE9Az7k/edit?usp=sharing). 

This features pure pursuit, lift PID, and intake anti jam.

## File description

### General `src/`
 * `autons.cpp`: autonomous routines and chassis constants
 * `main.cpp`:

### Subsystems `src/`
 * `intake.cpp`: intake code with jam detection and prevention
 * `lift.cpp`: lift code with presets and holds down

### Util `src/util/`
 * `pid.cpp`: PID class used for all PID everywhere and exit conditions
 * `util/cpp`: general math

### Drive `src/drive/`
 * `drive.cpp`: general drive functions and joystick curve modifier through controller
 * `exit_conditions.cpp`: finds out when robot is there
 * `pid_tasks.cpp`: point to point math that creates outputs to motors
 * `purepursuit_math.cpp`: path injection and smoothing
 * `set_pid.cpp`: setting PID and calculating paths
 * `slew.cpp`: ramps up max speed for the start of movements
 * `tracking.cpp`: calculates coordinates with motor encoders and IMU