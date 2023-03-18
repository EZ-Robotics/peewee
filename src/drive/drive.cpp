/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "drive.hpp"

#include "main.h"

// long get_raw_center() { return center_tracker.get_value(); }
// long get_raw_left() { return left_tracker.get_value(); }
// long get_raw_right() { return right_tracker.get_value(); }
long get_raw_left() { return l1.get_position(); }
long get_raw_right() { return r1.get_position(); }

// Returns inches
// double get_center() { return get_raw_center() / TICK_PER_INCH; }
double get_left() { return get_raw_left() / TICK_PER_INCH; }
double get_right() { return get_raw_right() / TICK_PER_INCH; }

// double get_left() { return get_raw_left() / TICK_PER_INCH; }
// double get_right() { return get_raw_right() / TICK_PER_INCH; }
// double get_angle() { return current.theta; }
double get_angle() { return imu.get_rotation() * IMU_SCALER; }

void reset_trackers() {
  // center_tracker.reset();
  // left_tracker.reset();
  // right_tracker.reset();
  l1.tare_position();
  r1.tare_position();
}

void set_angle(double input) {
  turnPID.set_target(input);
  angleppPID.set_target(input);
  headingPID.set_target(input);
  angle_rad = to_rad(input);
  imu.set_rotation(input);
  target.theta = input;
  current.theta = input;
}

// For internal use only
void raw_set_drive(int l, int r) {
  for (auto i : left_motors) {
    i.move_voltage(l * (12000.0 / 127.0));
  }
  for (auto i : right_motors) {
    i.move_voltage(r * (12000.0 / 127.0));
  }
}

// For external use only
void set_drive(int l, int r) {
  raw_set_drive(l, r);
}

// Brake drive
void drive_brake(pros::motor_brake_mode_e_t input) {
  for (auto i : left_motors) {
    i.set_brake_mode(input);
  }
  for (auto i : right_motors) {
    i.set_brake_mode(input);
  }
}

// Joystick deadzone
int deadzone(int input) {
  if (abs(input) > 3)
    return input;
  return 0;
}

// Input curve based on pilons
double inputcurve(int x) {
  double e = 2.718;
  double t = 1;
  return (powf(e, -(t / 10)) + powf(e, ((fabs(x) - 127) / 10)) * (1 - powf(e, -(t / 10)))) * x;
}

// Opcontrol
void tank_control() {
  set_drive(left_curve_function(deadzone(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y))), left_curve_function(deadzone(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y))));
  mode = DISABLED;
}

void arcade_control() {
  int l = deadzone(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) + inputcurve(deadzone(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)));
  int r = deadzone(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) - inputcurve(deadzone(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)));
  set_drive(l, r);
  mode = DISABLED;
}

void set_left_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {
  l_increase_.button = increase;
  l_decrease_.button = decrease;
}
void set_right_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {
  r_increase_.button = increase;
  r_decrease_.button = decrease;
}

// Increase / decrease left and right curves
void l_increase() { left_curve_scale += 0.1; }
void l_decrease() {
  left_curve_scale -= 0.1;
  left_curve_scale = left_curve_scale < 0 ? 0 : left_curve_scale;
}
void r_increase() { right_curve_scale += 0.1; }
void r_decrease() {
  right_curve_scale -= 0.1;
  right_curve_scale = right_curve_scale < 0 ? 0 : right_curve_scale;
}

// Button press logic for increase/decrease curves
void button_press(button_* input_name, int button, std::function<void()> change_curve) {
  // If button is pressed, increase the curve and set toggles.
  if (button && !input_name->lock) {
    change_curve();
    input_name->lock = true;
    input_name->release_reset = true;
  }

  // If the button is still held, check if it's held for 500ms.
  // Then, increase the curve every 100ms by 0.1
  else if (button && input_name->lock) {
    input_name->hold_timer += DELAY_TIME;
    if (input_name->hold_timer > 500.0) {
      input_name->increase_timer += DELAY_TIME;
      if (input_name->increase_timer > 100.0) {
        change_curve();
        input_name->increase_timer = 0;
      }
    }
  }

  // When button is released for 250ms, save the new curve value to the SD card
  else if (!button) {
    input_name->lock = false;
    input_name->hold_timer = 0;

    if (input_name->release_reset) {
      input_name->release_timer += DELAY_TIME;
      if (input_name->release_timer > 250.0) {
        input_name->release_timer = 0;
        input_name->release_reset = false;
      }
    }
  }
}

// Modify curves with button presses and display them to controller
void modify_curve_with_controller() {
  button_press(&l_increase_, master.get_digital(l_increase_.button), &l_increase);
  button_press(&l_decrease_, master.get_digital(l_decrease_.button), &l_decrease);
  if (!is_tank) {
    button_press(&r_increase_, master.get_digital(r_increase_.button), r_increase);
    button_press(&r_decrease_, master.get_digital(r_decrease_.button), r_decrease);
  }

  auto sr = std::to_string(right_curve_scale);
  auto sl = std::to_string(left_curve_scale);
  if (!is_tank)
    master.set_text(2, 0, sl + "   " + sr);
  else
    master.set_text(2, 0, sl);
}

// Left curve function
double left_curve_function(double x) {
  if (left_curve_scale != 0) {
    // if (CURVE_TYPE)
    return (powf(2.718, -(left_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(left_curve_scale / 10)))) * x;
    // else
    // return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return x;
}

// Right curve function
double right_curve_function(double x) {
  if (right_curve_scale != 0) {
    // if (CURVE_TYPE)
    return (powf(2.718, -(right_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(right_curve_scale / 10)))) * x;
    // else
    // return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return x;
}

void set_curve_default(double left, double right) {
  right_curve_scale = right;
  left_curve_scale = left;
}