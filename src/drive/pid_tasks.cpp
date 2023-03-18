/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"
#include "purepursuit_math.hpp"
#include "set_pid.hpp"
#include "util/util.hpp"

void auto_task() {
  while (true) {
    // Autonomous PID
    switch (mode) {
      case DRIVE:
        drive_pid_task();
        break;
      case TURN:
      case TURN_TO_POINT:
        turn_pid_task();
        break;
      case SWING:
      case SWING_TO_POINT:
        swing_pid_task();
        break;
      case TO_POINT:
        point_to_point();
        break;
      case PURE_PURSUIT:
        pure_pursuit();
        break;
      default:
        break;
    }

    AUTO_RAN = mode != DISABLE ? true : false;

    pros::delay(DELAY_TIME);
  }
}
pros::Task autoTask(auto_task);

void drive_pid_task() {
  // Compute PID
  leftPID.compute(get_left());
  rightPID.compute(get_right());
  // headingPID.compute(imu.get_rotation());
  aPID.compute(get_angle());

  // Clip output power
  int l_drive_out = clip_num(leftPID.output, max_xy, -max_xy);
  int r_drive_out = clip_num(rightPID.output, max_xy, -max_xy);
  int h_out = clip_num(aPID.output, 127, -127);

  printf("angle: %f   l: %f  r: %f  \n", get_angle(), get_left(), get_right());

  // Set motors
  // set_left(l_drive_out + h_out);
  // set_right(r_drive_out - h_out);
}

void turn_pid_task() {
  // Compute turn PID and find shortest path to angle
  // aPID.compute(imu.get_rotation());
  if (mode == TURN) {
    turnPID.compute(get_angle());
  } else {
    int add = current_turn_type == REV ? 180 : 0;
    double a_target = absolute_angle_to_point(turn_to_point_target, current) + add;
    turnPID.set_target(relative_angle_to_point(a_target));
    turnPID.compute(0);
  }

  int speed;
  if (fabs(turnPID.error) < 30)
    speed = 30;
  else
    speed = max_turn;

  // Clip output power
  int turn_out = clip_num(turnPID.output, speed, -speed);

  // Set motors
  set_drive(turn_out, -turn_out);
}

void swing_pid_task() {
  // Compute turn PID and find shortest path to angle
  // aPID.compute(imu.get_rotation());
  if (mode == SWING) {
    swingPID.compute(get_angle());
    printf("Swing tar%f    out %f  current %f  error %f\n", swingPID.target, swingPID.output, get_angle(), swingPID.error);
  } else {
    int add = current_turn_type == REV ? 180 : 0;
    double a_target = absolute_angle_to_point(turn_to_point_target, current) + add;
    swingPID.set_target(relative_angle_to_point(a_target));
    swingPID.compute(0);
  }

  // Clip output power
  int swing_out = clip_num(swingPID.output, max_turn, -max_turn);

  // Set motors
  if (current_swing == LEFT_SWING) {
    set_drive(swing_out, 0);
  } else {
    set_drive(0, -swing_out);
  }
}

void point_to_point() {
  // Flip the direction at the end of the motion so the robot can feedback
  turn_types used_dir = current_turn_type;
  if (passed_target && has_been_within_target) {
    // if (mode == TO_POINT)
    used_dir = used_dir == FWD ? REV : FWD;
    // printf("flipped\n");
  }

  // Add for direction
  int add = used_dir == REV ? 180 : 0;
  int dir = add == 180 ? -1 : 1;

  // Check to see if we've passed target
  pose b = vector_off_point(24, {movements.back().target.x, movements.back().target.y, a_target + 90});
  pose a = vector_off_point(24, {movements.back().target.x, movements.back().target.y, a_target - 90});
  pose c = current;
  int there = sgn(((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)));  // cross product to decide if above/below line

  if (there == 1)
    passed_target = current_turn_type == FWD ? true : false;
  else if (there == -1)
    passed_target = current_turn_type == REV ? true : false;

  printf("curr: (%.2f, %.2f)   a(%.2f, %.2f)  b(%.2f, %.2f)   passed: %i   there %i   dir %i   current turn type: %i\n", c.x, c.y, a.x, a.y, b.x, b.y, passed_target, there, dir, current_turn_type);

  // Clip max speed when around target
  if (fabs(distance_to_point(temp_xy_target, current)) < 1.0 && (pp_index >= movements.size() - 1 || mode == TO_POINT)) {
    // max_a = max_a > 50 ? 50 : max_a;
  }

  // Stop updating angle when within a radius of target
  if (fabs(distance_to_point(target, current)) > STOP_UPDATING_ANGLE || movements.size() - pp_index > 1) {
    a_target = absolute_angle_to_point(target, current) + add;
    temp_xy_target = target;
  } else {
    if (!has_been_within_target) {
      temp_xy_target = target;
      // temp_xy_target = vector_off_point(distance_to_point(movements[movements.size() - 1].target, current) * cos(to_rad(absolute_angle_to_point(movements[movements.size() - 1].target, current))), {current.x, current.y, current.theta});
    }
    has_been_within_target = true;
  }

  // Compute angle PID and find shortest path to angle
  aPID.set_target(relative_angle_to_point(a_target));
  aPID.compute(0);

  // Compute xy PID
  xyPID.set_target(distance_to_point(temp_xy_target, current) * dir);
  xyPID.compute(0);

  // Raw outputs
  double xy_raw_output = xyPID.output * cos(to_rad(relative_angle_to_point(a_target)));
  double a_raw_output = aPID.output;

  // Left/right outputs
  double l_output = xy_raw_output + a_raw_output;
  double r_output = xy_raw_output - a_raw_output;

  // Calculate max speed
  double l_slew_out = slew_calculate(left_slew, get_left());
  double r_slew_out = slew_calculate(right_slew, get_right());
  double slew_smallest = std::min(std::fabs(l_slew_out), std::fabs(r_slew_out));
  double smallest = std::min(slew_smallest, (double)max_xy);
  double ratio = std::max(std::fabs(l_output), std::fabs(r_output)) / smallest;
  if (ratio > 1) {
    l_output /= ratio;
    r_output /= ratio;
  }

  // printf("relative %f   absolute %f", )

  // printf("dir: %i   passed: %i   has been witin: %i   add: %i   xerr: %f   aerr: %f\n", dir, passed_target, has_been_within_target, add, xyPID.target, aPID.target);
  //  printf("tar (%f, %f)   angle %f\n", temp_xy_target.x, temp_xy_target.y, absolute_angle_to_point(movements[movements.size() - 1].target, current));
  //  printf("passed %i   outs(%f, %f)   aerr: %f  xyerr: %f      tar(%f, %f)  cur(%f, %f, %f)   pp_index %i\n", passed_target, xy_raw_output, a_raw_output, aPID.target, xyPID.target, temp_xy_target.x, temp_xy_target.y, current.x, current.y, current.theta, pp_index);
  //  printf("raw outs(%f, %f)  clipped outs(%i, %i)   max(%i, %i)\n", xy_raw_output, a_raw_output, l_output, r_output, max_xy, max_a);
  //  printf("distance: %f\n", distance_to_point(target, current) * dir);
  //  printf("outs(%i, %i)   rawxy %f   rawa %f  \n", l_output, r_output, xy_raw_output, a_raw_output);

  // printf("l,r(%f  %f)   min %f   max_xy %i   ratio %f\n", l_output, r_output, smallest, max_xy, ratio);

  // Set motors
  set_drive(l_output, r_output);
}

void pure_pursuit() {
  if (fabs(distance_to_point(movements[pp_index].target, current)) < LOOK_AHEAD) {
    if (pp_index < movements.size() - 1) {
      pp_index = pp_index >= movements.size() - 1 ? pp_index : pp_index + 1;
      raw_move_odom(movements[pp_index]);
    }
  }
  point_to_point();
}