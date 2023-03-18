/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

void set_x(double x) {
  target.x = x;
  current.x = x;
}
void set_y(double y) {
  target.y = y;
  current.y = y;
}
void set_theta(double a) { set_angle(a); }
void reset_odom() { set_pose({0, 0, 0}); }

void set_pose(pose itarget) {
  set_theta(itarget.theta);
  set_x(itarget.x);
  set_y(itarget.y);
}

// Tracking based on https://wiki.purduesigbots.com/software/odometry
void tracking_task() {
  double l_current = 0, r_current = 0;
  double c_current = 0;
  double l = 0, r = 0, c = 0;  // delta distance
  double l_last = 0, r_last = 0, c_last = 0;
  double radius_r = 0, radius_c = 0, h = 0, h2 = 0;  // rad for big circle
  double beta = 0, alpha = 0, theta = 0, last_theta = 0, current_global_theta = 0;
  double Xx = 0, Yy = 0, Xy = 0, Yx = 0;

  double encoder_theta = 0, encoder_angle_rad = 0;
  pros::delay(2500);
  while (true) {
    l_current = get_left();
    r_current = get_right();
    // c_current = get_raw_center();

    l = l_current - l_last;
    r = r_current - r_last;
    // c = c_current - c_last;

    l_last = l_current;
    r_last = r_current;
    // c_last = c_current;

    double width = LEFT_OFFSET + RIGHT_OFFSET;

    // diff between wheels for correcting turning
    encoder_theta = (l - r) / width;  // encoder theta
    current_global_theta = to_rad(get_angle());
    theta = current_global_theta - last_theta;  // imu theta
    last_theta = current_global_theta;

    if (theta != 0) {
      radius_r = r / theta;
      beta = theta / 2.0;
      h = ((radius_r + RIGHT_OFFSET) * sin(beta)) * 2.0;
      // radius_c = c / theta;
      // h2 = (radius_c + CENTER_OFFSET) * 2.0 * sin(beta);
    } else {
      h = l;
      // h2 = 0;
      beta = 0;
    }

    alpha = angle_rad + beta;

    // Xx = h2 * cos(alpha);
    // Xy = h2 * -sin(alpha);
    Yx = h * sin(alpha);
    Yy = h * cos(alpha);

    current.x += (Xx + Yx);
    current.y += (Xy + Yy);
    angle_rad += theta;
    encoder_angle_rad += encoder_theta;
    current.theta = to_deg(encoder_angle_rad);
    // current.theta = current_global_theta;

    pros::delay(1);
  }
}