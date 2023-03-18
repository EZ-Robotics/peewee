/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

// Set minimum power
void set_slew_min_power(int fwd, int rev) {
  SLEW_MIN_POWER[0] = abs(fwd);
  SLEW_MIN_POWER[1] = abs(rev);
}

// Set distance to slew for
void set_slew_distance(int fwd, int rev) {
  SLEW_DISTANCE[0] = abs(fwd);
  SLEW_DISTANCE[1] = abs(rev);
}

// Initialize slew
void slew_initialize(slew_ &input, bool slew_on, double max_speed, double current_sensor, bool backwards) {
  input.enabled = slew_on;
  input.max_speed = max_speed;

  input.sign = backwards ? -1 : 1;
  input.x_intercept = current_sensor + ((SLEW_DISTANCE[backwards] * input.sign));
  input.y_intercept = max_speed * input.sign;
  input.slope = ((input.sign * SLEW_MIN_POWER[backwards]) - input.y_intercept) / (input.x_intercept - 0 - current_sensor);  // y2-y1 / x2-x1
}

// Slew calculation
double slew_calculate(slew_ &input, double current) {
  // Is slew still on?
  if (input.enabled) {
    // Error is distance away from completed slew
    input.error = input.x_intercept - current;

    // When the sign of error flips, slew is completed
    if (sgn(input.error) != input.sign)
      input.enabled = false;

    // Return y=mx+b
    else if (sgn(input.error) == input.sign)
      return ((input.slope * input.error) + input.y_intercept) * input.sign;
  }
  // When slew is completed, return max speed
  return max_xy;
}
