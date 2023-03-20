/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include "api.h"

enum lift_state {
  DOWN = 0,
  MID = 50,
  UP = 830
};

inline pros::Motor lift_motor(6, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

void set_lift_exit();
void set_lift_speed(int input);
void set_lift(int input);
void reset_lift();
void set_lift_state(lift_state input);
void wait_lift();

void lift_opcontrol();