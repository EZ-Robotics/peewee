steps to tuning

setup.hpp do drive ports
setup.hpp do imu ports
setup cart, wheel size, and gear ratio in `autons.cpp`

now we have to tune the drive width
in `drive.cpp` change `get_angle` to return `current.theta()`
run an auton that turns to 3600
adjust `WIDTH` at the top of `autons.cpp` until the robot turns exactly 10 times

now we have to tune imu scaler 
in `drive.cpp` change `get_angle()` to return `imu.get_rotation()` (this will stay like this now)
run an auton that turns to 3600
adjust `IMU_SCALER` until the robot turns exactly 10 times