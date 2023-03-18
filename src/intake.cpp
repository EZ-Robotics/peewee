/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

using namespace ez;

int target_speed = 0;  // Global target speed

// For use in this file only
void raw_set_intake(int input) {
  for (auto i : intake_motors) {
    i.move_voltage(input * (12000.0 / 127.0));
  }
}

// This is used outside of this file
void set_intake(int input) {
  raw_set_intake(input);
  target_speed = input;
}

// Intake task with antijam logic
void intake_task() {
  const int wait_time = 30;
  const int outtake_time = 250;
  int jam_counter = 0;
  bool is_jammed = false;

  while (true) {
    // Run intake full power in opposite direction for outtake_time ms when jammed, then
    // set intake back to normal
    if (is_jammed) {
      raw_set_intake(-127 * sgn(target_speed));
      jam_counter += DELAY_TIME;
      if (jam_counter > outtake_time) {
        is_jammed = false;
        jam_counter = 0;
        raw_set_intake(target_speed);
      }
    }

    // Detect a jam if velocity is 0 for wait_time ms
    else if (target_speed != 0 && intake_motors[0].get_actual_velocity() == 0) {
      jam_counter += DELAY_TIME;
      if (jam_counter > wait_time) {
        jam_counter = 0;
        is_jammed = true;
      }
    }

    // Reset jam_counter when button is released
    if (target_speed == 0) {
      jam_counter = 0;
    }

    pros::delay(DELAY_TIME);
  }
}
pros::Task Intake_Task(intake_task);

// Opcontrol
void intake_opcontrol() {
  if (master.get_digital(DIGITAL_R1))
    set_intake(127);
  else if (master.get_digital(DIGITAL_R2))
    set_intake(-127);
  else
    set_intake(0);
}
