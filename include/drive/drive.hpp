/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>

#include "api.h"

inline bool is_tank = true;

// long get_raw_center();
long get_raw_left();
long get_raw_right();

// double get_center();
double get_left();
double get_right();

double get_angle();
void reset_trackers();
void set_angle(double input);

void raw_set_drive(int l, int r);
void set_drive(int l, int r);
void drive_brake(pros::motor_brake_mode_e_t input);
int deadzone(int input);

void tank_control();
void arcade_control();

/**
 * Save input to sd card
 */
void save_l_curve_sd();
void save_r_curve_sd();

/**
 * Struct for buttons for increasing/decreasing curve with controller
 */
struct button_ {
  bool lock = false;
  bool release_reset = false;
  int release_timer = 0;
  int hold_timer = 0;
  int increase_timer;
  pros::controller_digital_e_t button;
};

inline button_ l_increase_;
inline button_ l_decrease_;
inline button_ r_increase_;
inline button_ r_decrease_;

/**
 * Function for button presses.
 */
void button_press(button_ *input_name, int button, std::function<void()> changeCurve);

/**
 * The left and right curve scalers.
 */
inline double left_curve_scale;
inline double right_curve_scale;

/**
 * Increase and decrease left and right curve scale.
 */
void l_decrease();
void l_increase();
void r_decrease();
void r_increase();

/**
 * Sets the default joystick curves.
 *
 * \param left
 *        Left default curve.
 * \param right
 *        Right default curve.
 */
void set_curve_default(double left, double right = 0);

/**
 * Sets buttons for modifying the left joystick curve.
 *
 * \param decrease
 *        a pros button enumerator
 * \param increase
 *        a pros button enumerator
 */
void set_left_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

/**
 * Sets buttons for modifying the right joystick curve.
 *
 * \param decrease
 *        a pros button enumerator
 * \param increase
 *        a pros button enumerator
 */
void set_right_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

void modify_curve_with_controller();
double left_curve_function(double x);
double right_curve_function(double x);
