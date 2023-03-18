/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "util/util.hpp"

void set_x(double x);
void set_y(double y);
void set_theta(double a);
void reset_odom();
void set_pose(pose itarget);

inline pose target;
inline pose current;
inline double angle_rad;

void tracking_task();
inline pros::Task trackingTask(tracking_task);