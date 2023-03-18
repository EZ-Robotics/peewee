A robot for the SPUR-FLYS minigame, [Mini-Up](https://docs.google.com/document/d/1mCmWGhqI1ZUdg05hKQo3zhatCqkk73Puq9dKWE9Az7k/edit?usp=sharing). 

This features pure pursuit, lift PID, and intake anti jam.  It uses [EZ-GUI](https://github.com/EZ-Robotics/EZ-GUI) to see motor temperatures on the brain and autonomous selector. 

## Resources  
[BLRS Wiki PID](https://wiki.purduesigbots.com/software/control-algorithms/pid-controller)  
[George Gillard PID](https://georgegillard.com/resources/documents)  
[Tracking by 5225A](https://wiki.purduesigbots.com/software/odometry)  
[Pure Pursuit by Sarah](https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit)  
[Adaptive Pure Pursuit by DAWGMA](https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552)  
[Path Smoothing by James Teow](https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4)  
[LemLib by Liam 1010](https://github.com/LemLib/LemLib)  
[The Beauty of Bézier Curves by Freya Holmér](https://youtu.be/aVwxzDHniEw)

## File description

### General `src/`
 * `autons.cpp` autonomous routines and chassis constants
 * `main.cpp` main function calls

### Subsystems `src/`
 * `intake.cpp` intake code with jam detection and prevention
 * `lift.cpp` lift code with presets and holds down

### Util `src/util/`
 * `pid.cpp` PID class used for all PID everywhere and exit conditions
 * `util/cpp` general math

### Drive `src/drive/`
 * `drive.cpp` general drive functions and joystick curve modifier through controller
 * `exit_condiions.cpp` finds out when robot is there
 * `pid_tasks.cpp` point to point math that creates outputs to motors
 * `purepursuit_math.cpp` path injection and smoothing
 * `set_pid.cpp` setting PID and calculating paths
 * `slew.cpp` ramps up max speed for the start of movements
 * `tracking.cpp` calculates coordinates with motor encoders and IMU