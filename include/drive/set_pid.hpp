/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "util/pid.hpp"

inline bool passed_target = false;
inline double a_target = 0;
inline int pp_index = 0;
inline int max_xy = MAX_XY;
inline int max_turn = TURN_SPEED;
inline pose temp_xy_target;
inline bool is_close = false;
inline pose turn_to_point_target = {0, 0};
inline turn_types current_turn_type = FWD;
inline std::vector<int> injected_pp_index;
inline std::vector<odom> movements;

inline swing_types current_swing;

inline PID turnPID(0);
inline PID headingPID(0);
inline PID angleppPID(0);
inline PID aPID(0);
inline PID swingPID(0);

inline PID leftPID(0);
inline PID rightPID(0);

inline PID xyPID(0);
inline PID xyendPID(0);
inline PID ppPID(0);

inline PID ptp_xy_pid(0);
inline PID ptp_heading_pid(0);

void reset_pid_targets();
void raw_move_odom(odom imovement);

void drive_pid(double target, int speed = MAX_XY);
void turn_pid(double itarget, int speed = TURN_SPEED);
void turn_pid(pose itarget, turn_types dir, int speed = TURN_SPEED);
void swing_pid(swing_types swing, double itarget, int speed = TURN_SPEED);
void swing_pid(swing_types swing, pose itarget, turn_types dir, int speed = TURN_SPEED);

void relative_ptp(double distance, int speed = MAX_XY);
void ptp(odom imovement);
void pure_pursuit(std::vector<odom> imovements);
void injected_pp(std::vector<odom> imovements);
void smooth_pp(std::vector<odom> imovements, double weight_smooth = 0.75, double weight_data = 0.05, double tolerance = 0.0001);
void ptp_at_angle(odom imovement);