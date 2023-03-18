/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

const int SLOW_SPEED = 100;  // Speed when lift is going down
const int FAST_SPEED = 127;  // Speed when lift is going up

PID liftPID{4, 0, 0, 0, "Lift"};  // lift object

// Set used max speed
int lift_max_speed = FAST_SPEED;
void set_lift_speed(int input) {
  lift_max_speed = abs(input);
}

void set_lift(int input) { lift_motor = input; }   // Set lift to voltage
void reset_lift() { lift_motor.tare_position(); }  // Reset sensor

// Exit condition constants
void set_lift_exit() {
  liftPID.set_exit_condition(80, 20, 300, 50, 500, 500);
}

// Print states
std::string lift_state_to_string(lift_state input) {
  switch (input) {
    case DOWN:
      return "Down";
      break;
    case MID:
      return "Mid";
      break;
    case UP:
      return "Up";
      break;
    default:
      return "Out of bounds lift state";
      break;
  }
}

// Set new state
lift_state current_lift_state;
void set_lift_state(lift_state input) {
  set_lift_speed(current_lift_state > input ? SLOW_SPEED : FAST_SPEED);  // Update max speed
  current_lift_state = input;
  liftPID.set_target(input);
  std::cout << "\nNew Lift State: " << lift_state_to_string(input);
}

// Lift task
void liftTask() {
  double output = 0;
  long timer = 0;
  bool did_reset = false;
  while (true) {
    // Run PID and clip output to max speed
    double current = lift_motor.get_position();
    double clipped_pid = clip_num(liftPID.compute(current), lift_max_speed, -lift_max_speed);

    // When the lift is going down and is close to all the way down, hold a little power down.
    // Reset the lift sensor when it reaches the very bottom
    if (current_lift_state == DOWN) {
      if (current >= 20)
        output = clipped_pid;
      else {
        // Lift has to be within 20deg and have a velocity of 0 for 250ms to reset the sensor
        bool check = (lift_motor.get_actual_velocity() == 0 && !pros::competition::is_disabled()) ? true : false;
        if (check) timer += DELAY_TIME;
        if (timer >= 250) {
          output = -3;
          if (!did_reset) reset_lift();
          did_reset = true;
          timer = 250;
        } else {
          output = -40;
        }
      }
    }

    // If the lit isn't going down, set output to PID output
    else {
      timer = 0;
      did_reset = false;
      output = clipped_pid;
    }

    if (pros::competition::is_disabled()) timer = 0;

    set_lift(output);

    pros::delay(DELAY_TIME);
  }
}
pros::Task lift_task(liftTask);

// Blocking function for lift, used during autonomous
void wait_lift() {
  while (liftPID.exit_condition(lift_motor, true) == RUNNING) {
    pros::delay(DELAY_TIME);
  }
}

// Opcontrol
void lift_opcontrol() {
  if (master.get_digital_new_press(DIGITAL_L1)) {
    if (current_lift_state == UP || current_lift_state == MID)
      set_lift_state(DOWN);
    else
      set_lift_state(UP);
  }

  // if (master.get_digital_new_press(DIGITAL_R2) && last_r2 == 0)
  //   set_lift_state(MID);
}