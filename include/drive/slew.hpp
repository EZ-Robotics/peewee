/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

/**
 * Constants for slew
 */
inline double SLEW_DISTANCE[2];
inline double SLEW_MIN_POWER[2];

// Slew
struct slew_ {
  int sign = 0;
  double error = 0;
  double x_intercept = 0;
  double y_intercept = 0;
  double slope = 0;
  double output = 0;
  bool enabled = false;
  double max_speed = 0;
};

inline slew_ left_slew;
inline slew_ right_slew;

void slew_initialize(slew_ &input, bool slew_on, double max_speed, double current_sensor, bool backwards);
double slew_calculate(slew_ &input, double current);