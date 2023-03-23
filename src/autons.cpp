/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "drive/exit_conditions.hpp"
#include "drive/pid_tasks.hpp"
#include "drive/purepursuit_math.hpp"
#include "drive/set_pid.hpp"
#include "main.h"
#include "util/util.hpp"

// Max speeds
const int MAX_XY = 110;
const int TURN_SPEED = 90;

// Pure pursuit constants
const double STOP_UPDATING_ANGLE = 3;                  // When looking at a target, stop updating the angle when target is within this
const double LOOK_AHEAD = STOP_UPDATING_ANGLE + 0.25;  // Pure pursuit look ahead distance
const double SPACING = 2.0;                            // Spacing for pure pursuit injected points

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

const int INTAKE_HOLDING = 10;

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

void score() {
  set_lift_state(UP);
  wait_lift();
  set_intake(-127);
  pros::delay(2000);
  set_intake(127);
  set_lift_state(DOWN);
  wait_lift();
  set_intake(INTAKE_HOLDING);
}

void skills() {
  set_intake(127);
  injected_pp({{{0, 15}, FWD}});
  wait_drive();

  std::vector<pose> carrot = boomerang(current, {18, -3, 90});
  smooth_pp({{carrot[0], REV},
             {carrot[1], REV},
             {{69.5, carrot[1].y + 3.5}, REV}});
  // print_path_for_python(movements);
  wait_drive();

  score();

  smooth_pp({{{94, -7}, REV}});
  wait_drive();

  carrot = boomerang(current, {91, 18, 0});
  smooth_pp({{carrot[0], FWD},
             {carrot[1], FWD},
             {{carrot[1].x, 90}, FWD},
             {{carrot[1].x, 105}, FWD, 40}});
  pp_wait_until(2);
  set_intake(127);
  wait_drive();

  std::vector<pose> carrot2 = boomerang(current, {carrot[1].x - 4, 22, 0});
  smooth_pp({{{carrot[1].x + 3, 30}, REV},
             {carrot2[0], REV},
             {carrot2[1], REV}});
  wait_drive();
}

void auton1() {
  set_pose({93, 105, 0});
  smooth_pp({{{90, 105 - 12}, REV}});
  wait_drive();
  // pure_pursuit(pointsAlongArc(target, -90, FWD, MAX_XY, 15));
  //  turn_pid(3600, 40);
  //  std::vector<pose> carrot1 = boomerang(current, {0, 15, 45});
  //  std::vector<pose> carrot2 = boomerang(carrot1[1], {30, 0, 135});
  //  smooth_pp({{carrot1[0], FWD, 80},
  //             {carrot1[1], FWD, 80},
  //             {carrot2[0], FWD, 80},
  //             {carrot2[1], FWD, 80}});

  double dist = 15;
  int speed = 70;
  // std::vector<odom> printing = {{{24, 0}, REV, 70}};
  // std::vector<odom> printing = {{{0, dist}, FWD, speed},
  //                              {{dist, dist}, FWD, speed},
  //                              {{dist, 0}, FWD, speed},
  //                              {{0, 0}, FWD, speed}};
  // printing = inject_points(printing);
  // printing = smooth_path(printing);
  // printing = update_path_angles(printing);
  // print_path_for_python(printing);

  // std::vector<odom> printing = {{{0, dist}, FWD, speed},
  //                               {{dist, dist}, FWD, speed},
  //                               {{dist, 0}, FWD, speed},
  //                               {{0, 0}, FWD, speed}};

  // std::vector<odom> smooth_path(std::vector<odom> ipath, double weight_smooth = 0.75, double weight_data = 0.05, double tolerance = 0.0001);
  // printing = smooth_path(inject_points(printing), 0.75, 0.05, 0.0001);

  // smooth_pp({{{0, dist}, FWD, speed},
  //            {{dist, dist}, FWD, speed},
  //            {{dist, 0}, FWD, speed},
  //            {{0, 0}, FWD, speed},
  //            {{0, dist}, FWD, speed}});
  // wait_drive();

  // smooth_pp({{{12, 12}, REV}});

  // smooth_pp({{{0, 12}, FWD, speed}});
  // wait_drive();
  //    injected_pp({{{0, 18}, FWD}});
  //       injected_pp({{{0, 18}, FWD, 40}});
  //    while (current.y < 18) { pros::delay(10); }
  //    wait_drive();
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
