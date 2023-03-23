/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "set_pid.hpp"

#include "drive.hpp"
#include "main.h"
#include "purepursuit_math.hpp"
#include "setup.hpp"
#include "util/util.hpp"

void reset_pid_targets() {
  turnPID.reset_variables();

  xyPID.reset_variables();
  aPID.reset_variables();
}

// Set turn PID, for external use
void turn_pid(double itarget, int speed) {
  // Print targets
  printf("Turn PID Started... Target Degree: %f ", itarget);
  printf("\n");

  max_turn = abs(speed);
  target.theta = itarget;
  turnPID.set_target(itarget);
  headingPID.set_target(itarget);
  aPID.set_target(itarget);  // this should get deleted at some point

  mode = TURN;
}

// Set turn PID, for external use
void turn_pid(pose itarget, turn_types dir, int speed) {
  // Print targets
  printf("Turn to Point PID Started... Target Point: (%f, %f) ", itarget.x, itarget.y);
  printf("\n");

  turn_to_point_target = itarget;
  current_turn_type = dir;

  int add = current_turn_type == REV ? 180 : 0;
  target.theta = absolute_angle_to_point(turn_to_point_target, current) + add;

  max_turn = abs(speed);

  mode = TURN_TO_POINT;
}

// Set turn PID, for external use
void swing_pid(swing_types swing, double itarget, int speed) {
  // Print targets
  printf("Swing PID Started... Target Degree: %f ", itarget);
  printf("\n");

  max_turn = abs(speed);
  target.theta = itarget;
  turnPID.set_target(itarget);
  headingPID.set_target(itarget);
  swingPID.set_target(itarget);
  aPID.set_target(itarget);  // this should get deleted at some point

  current_swing = swing;

  mode = SWING;
}

// Set turn PID, for external use
void swing_pid(swing_types swing, pose itarget, turn_types dir, int speed) {
  // Print targets
  printf("Swing to Point PID Started... Target Point: (%f, %f) ", itarget.x, itarget.y);
  printf("\n");

  turn_to_point_target = itarget;
  current_turn_type = dir;

  int add = current_turn_type == REV ? 180 : 0;
  target.theta = absolute_angle_to_point(turn_to_point_target, current) + add;

  max_turn = abs(speed);

  current_swing = swing;

  mode = SWING_TO_POINT;
}

// For internal use
void raw_move_odom(odom imovement) {
  // Update current turn type
  current_turn_type = imovement.turn_type;

  // Only update angle if the new target is unique
  if (imovement.target.x != target.x && imovement.target.y != target.y)
    target.theta = absolute_angle_to_point(imovement.target, target);

  // Check if the pp is between motions or at the end of the motion
  // and change constants accordingly
  if (pp_index >= movements.size() - 1 - (LOOK_AHEAD / SPACING) || mode == TO_POINT || is_close) {
    auto consts = xyendPID.get_constants();
    xyPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
  } else {
    auto consts = ppPID.get_constants();
    xyPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
  }

  auto a_consts = ptp_heading_pid.get_constants();
  aPID.set_constants(a_consts.kp, a_consts.ki, a_consts.kd, a_consts.start_i);

  if (mode == TO_POINT) {
    auto x_consts = ptp_xy_pid.get_constants();
    auto a_consts = ptp_heading_pid.get_constants();
    xyPID.set_constants(x_consts.kp, x_consts.ki, x_consts.kd, x_consts.start_i);
    aPID.set_constants(a_consts.kp, a_consts.ki, a_consts.kd, a_consts.start_i);
    // printf("constants changed\n");
  }

  // Set targets
  target.x = imovement.target.x;
  target.y = imovement.target.y;
  temp_xy_target = target;

  // Max speeds
  max_xy = abs(imovement.max_xy_speed);
}

void update_angle() {
  // Update angle target
  // target = movements[0].target;
  int add = current_turn_type == REV ? 180 : 0;
  int index = mode == TO_POINT ? 0 : 1;
  a_target = absolute_angle_to_point(movements[index].target, current) + add;
}

// Move point to point, for external use
void ptp(odom imovement) {
  // Print targets
  printf("Odom Motion Started... Target Coordinates: (%f, %f, %f) \n", imovement.target.x, imovement.target.y, imovement.target.theta);

  // Update angle
  movements.clear();
  movements.push_back(imovement);
  passed_target = false;
  is_close = false;  // Reset flag for being within range to stop updating

  // Run point_to_point()
  mode = TO_POINT;

  // Set new targets and update target angle
  double angle = absolute_angle_to_point(imovement.target, target);
  imovement.target.theta = angle;
  raw_move_odom(imovement);
  update_angle();

  // Initialize slew
  slew_initialize(left_slew, true, max_xy, get_left(), imovement.turn_type);
  slew_initialize(right_slew, true, max_xy, get_right(), imovement.turn_type);
}

// Pure pursuit, for external use
void pure_pursuit(std::vector<odom> imovements) {
  // Print targets
  printf("Pure Pursuit Motion Started... Target Coordinates: \n");
  for (int i = 0; i < imovements.size(); i++) {
    std::string turn = turn_types_to_string(imovements[i].turn_type);
    std::cout << "Point " << i << ": (" << imovements[i].target.x << ", " << imovements[i].target.y << ", " << imovements[i].target.theta << ")  Turn: " << turn << "\n";
  }

  // Reset indexes and previous movements
  movements.clear();
  injected_pp_index.clear();
  pp_index = 0;
  passed_target = false;
  is_close = false;  // Reset flag for being within range to stop updating

  // This is used for pp_wait_until()
  for (int i = 0; i < imovements.size(); i++) {
    injected_pp_index.push_back(i);
  }

  // Set new targets and update path angles
  movements = update_path_angles(imovements);

  // Update angle target
  update_angle();

  // Run pure_pursuit()
  mode = PURE_PURSUIT;
  raw_move_odom(movements[0]);
}

// Pure pursuit, for external use
void injected_pp(std::vector<odom> imovements) {
  // Print targets
  printf("Point Injected Pure Pursuit Motion Started... Target Coordinates: \n");
  for (int i = 0; i < imovements.size(); i++) {
    std::string turn = turn_types_to_string(imovements[i].turn_type);
    std::cout << "Point " << i << ": (" << imovements[i].target.x << ", " << imovements[i].target.y << ", " << imovements[i].target.theta << ")  Turn: " << turn << "\n";
  }

  // Reset indexes and previous movements
  movements.clear();
  injected_pp_index.clear();
  pp_index = 0;
  passed_target = false;
  is_close = false;  // Reset flag for being within range to stop updating

  // Set new targets
  movements = inject_points(imovements);

  raw_move_odom(movements[0]);

  // Update angle target
  update_angle();

  /*
  // Print subpoints
  printf("Subpoints\n");
  for (int i = 0; i < movements.size(); i++) {
    std::string turn = turn_types_to_string(movements[i].turn_type);
    std::cout << "Point " << i << ": (" << movements[i].target.x << ", " << movements[i].target.y << ", " << movements[i].target.theta << ")  Turn: " << turn << "\n";
  }
  */

  // Run pure_pursuit()
  mode = PURE_PURSUIT;
}

// Pure pursuit, for external use
void smooth_pp(std::vector<odom> imovements, double weight_smooth, double weight_data, double tolerance) {
  // Print targets
  printf("Smooth Point Injected Pure Pursuit Motion Started... Target Coordinates: ");
  for (int i = 0; i < imovements.size(); i++) {
    std::string turn = turn_types_to_string(imovements[i].turn_type);
    std::cout << "Point " << i << ": (" << imovements[i].target.x << ", " << imovements[i].target.y << ", " << imovements[i].target.theta << ")  Turn: " << turn << "\n";
  }

  // Reset indexes and previous movements
  movements.clear();
  injected_pp_index.clear();
  pp_index = 0;
  passed_target = false;
  is_close = false;  // Reset flag for being within range to stop updating

  // Set new targets
  movements = smooth_path(inject_points(imovements), weight_smooth, weight_data, tolerance);

  raw_move_odom(movements[0]);

  // Update angle target
  update_angle();
  /*
  // Print subpoints
  printf("Subpoints\n");
  for (int i = 0; i < movements.size(); i++) {
    std::string turn = turn_types_to_string(movements[i].turn_type);
    std::cout << "Point " << i << ": (" << movements[i].target.x << ", " << movements[i].target.y << ", " << movements[i].target.theta << ")  Turn: " << turn << "\n";
  }
  */

  // Run pure_pursuit()
  mode = PURE_PURSUIT;
}

///
// Fancier functions that use the above
///

// Relative odom
void relative_ptp(double distance, int speed) {
  // Calculate x,y based on distance (hypot)
  pose output;
  if (mode == SWING || mode == SWING_TO_POINT) {
    output = vector_off_point(distance, {current.x, current.y, target.theta});
  } else {
    output = vector_off_point(distance, {target.x, target.y, target.theta});
  }
  output.theta = target.theta;

  // Print targets
  printf("Relative Odom Motion Started... Distance: %.2f  Target Coordinates: (%f, %f, %f) \n", distance, output.x, output.y, output.theta);

  // Run raw odom
  ptp({output, sgn(distance) == 1 ? FWD : REV, speed});
  // injected_pp({{output, sgn(distance) == 1 ? FWD : REV, speed, MAX_A}});
}

// Ptp at angle
void ptp_at_angle(odom imovement) {
  printf("Pose to Pose through Smooth Pure Pursuit... Target Pose: (%f, %f, %f)\n", imovement.target.x, imovement.target.y, imovement.target.theta);
  std::vector<pose> carrot = boomerang(current, {imovement.target.x, imovement.target.y, imovement.target.theta});
  smooth_pp({{carrot[0], imovement.turn_type, imovement.max_xy_speed},
             {carrot[1], imovement.turn_type, imovement.max_xy_speed}});
}