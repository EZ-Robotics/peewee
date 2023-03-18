/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "drive/exit_conditions.hpp"
#include "drive/purepursuit_math.hpp"
#include "drive/set_pid.hpp"
#include "main.h"
#include "util/util.hpp"

// Max speeds
const int MAX_XY = 100;
const int TURN_SPEED = 90;

// Pure pursuit constants
const double STOP_UPDATING_ANGLE = 5;                  // When looking at a target, stop updating the angle when target is within this
const double LOOK_AHEAD = STOP_UPDATING_ANGLE + 0.25;  // Pure pursuit look ahead distance
const double SPACING = 1.0;                            // Spacing for pure pursuit injected points

// Motor rpm
const int CART = 600;
const double RATIO = 1.0;                                   // wheel / motor
const int TICK_PER_REV = (50.0 * (3600.0 / CART)) * RATIO;  // 2048 for trackers
const double WHEEL_DIA = 2.0;

// Drive dimensions
const double WIDTH = 7.2;
const double RIGHT_OFFSET = WIDTH / 2.0;
const double LEFT_OFFSET = WIDTH / 2.0;
const double IMU_SCALER = 1.005;

void set_pid_defaults() {
  // These are used for odometry motions
  xyPID.set_exit_condition(300, 1, 625, 3, 750, 750);
  aPID.set_exit_condition(100, 3, 500, 7, 500, 500);
  turnPID.set_exit_condition(100, 3, 500, 7, 500, 500);
  swingPID.set_exit_condition(100, 3, 500, 7, 500, 500);

  SLEW_MIN_POWER[FWD] = 60;
  SLEW_MIN_POWER[REV] = 60;

  SLEW_DISTANCE[FWD] = 8;
  SLEW_DISTANCE[REV] = 8;

  // BETTER STUFF FOR PUSHBOT
  turnPID.set_constants(3, 0.002, 18, 30);  // for turning

  // ptp constants
  ptp_xy_pid.set_constants(7, 0, 20, 0);
  ptp_heading_pid.set_constants(3, 0, 40, 0);  // (6, 0, 10, 0); (8, 0, 13, 0);

  // pp constants
  ppPID.set_constants(15, 0, 40, 0);    // injected pp constants
  xyendPID.set_constants(7, 0, 30, 0);  // ending pp constants
}

void auton1() {
  // turn_pid(3600, 40);
  // std::vector<pose> carrot1 = boomerang(current, {0, 15, 45});
  // std::vector<pose> carrot2 = boomerang(carrot1[1], {30, 0, 135});
  // smooth_pp({{carrot1[0], FWD, 80},
  //            {carrot1[1], FWD, 80},
  //            {carrot2[0], FWD, 80},
  //            {carrot2[1], FWD, 80}});

  // std::vector<odom> printing = {{{0, 0}, REV, 70}};
  // printing = inject_points(printing);
  double dist = 20;
  int speed = 40;
  // std::vector<odom> printing = {{{0, dist}, FWD, speed},
  //                               {{dist, dist}, FWD, speed},
  //                               {{dist, 0}, FWD, speed},
  //                               {{0, 0}, FWD, speed}};

  // std::vector<odom> smooth_path(std::vector<odom> ipath, double weight_smooth = 0.75, double weight_data = 0.05, double tolerance = 0.0001);
  // printing = smooth_path(inject_points(printing), 0.75, 0.05, 0.0001);
  // print_path_for_python(printing);

  smooth_pp({{{0, dist}, FWD, speed},
             {{dist, dist}, FWD, speed},
             {{dist, 0}, FWD, speed},
             {{0, 0}, FWD, speed},
             {{0, dist}, FWD, speed}});
  wait_drive();

  // smooth_pp({{{12, 12}, REV}});

  injected_pp({{{0, 0}, REV, speed}});
  wait_drive();
  //  injected_pp({{{0, 18}, FWD}});
  //     injected_pp({{{0, 18}, FWD, 40}});
  //  while (current.y < 18) { pros::delay(10); }
  //  wait_drive();
  /*
  turn_pid(45);
  wait_drive();

  turn_pid(-45);
  wait_drive();

  turn_pid(0);
  wait_drive();
  */
  // ptp({{0, 0}, REV});
  // injected_pp({{{0, 0}, REV}});
  // wait_drive();
}
