/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"
#include "util/util.hpp"

// User wrapper for exit condition
void wait_drive() {
  // Let the PID run at least 1 iteration
  pros::delay(DELAY_TIME);

  if (mode == TO_POINT || mode == PURE_PURSUIT) {
    exit_output xy_exit = RUNNING;
    exit_output a_exit = RUNNING;

    // Wait until pure pursuit is on the last point
    if (mode == PURE_PURSUIT) {
      while (pp_index != movements.size() - 1) {
        xy_exit = xy_exit != RUNNING ? xy_exit : xyPID.exit_condition({drive_motors});
        a_exit = a_exit != RUNNING ? a_exit : aPID.exit_condition({drive_motors});

        if (xy_exit == mA_EXIT || xy_exit == VELOCITY_EXIT || a_exit == mA_EXIT || a_exit == VELOCITY_EXIT)
          break;

        pros::delay(DELAY_TIME);
      }
    }

    while (xy_exit == RUNNING || a_exit == RUNNING) {
      xy_exit = xy_exit != RUNNING ? xy_exit : xyPID.exit_condition({drive_motors});
      a_exit = a_exit != RUNNING ? a_exit : aPID.exit_condition({drive_motors});
      pros::delay(DELAY_TIME);
    }
    std::cout << "  XY: " << exit_to_string(xy_exit) << " Exit.   A: " << exit_to_string(a_exit) << " Exit. \n";

    // if (x_exit == mA_EXIT || x_exit == VELOCITY_EXIT || y_exit == mA_EXIT || y_exit == VELOCITY_EXIT) {
    //   interfered = true;
    // }
  }

  // Drive Exit
  if (mode == DRIVE) {
    exit_output left_exit = RUNNING;
    exit_output right_exit = RUNNING;
    while (left_exit == RUNNING || right_exit == RUNNING) {
      left_exit = left_exit != RUNNING ? left_exit : leftPID.exit_condition(left_motors);
      right_exit = right_exit != RUNNING ? right_exit : rightPID.exit_condition(right_motors);
      pros::delay(DELAY_TIME);
    }
    std::cout << "  Left: " << exit_to_string(left_exit) << " Exit.   Right: " << exit_to_string(right_exit) << " Exit.\n";

    // if (left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
    //   interfered = true;
    // }
  }

  // Turn Exit
  else if (mode == TURN || mode == TURN_TO_POINT) {
    exit_output turn_exit = RUNNING;
    while (turn_exit == RUNNING) {
      turn_exit = turn_exit != RUNNING ? turn_exit : turnPID.exit_condition({drive_motors});
      pros::delay(DELAY_TIME);
    }
    std::cout << "  Turn: " << exit_to_string(turn_exit) << " Exit.\n";

    // if (a_exit == mA_EXIT || a_exit == VELOCITY_EXIT) {
    //   interfered = true;
    // }
  }

  // Swing Exit
  else if (mode == SWING || mode == SWING_TO_POINT) {
    exit_output swing_exit = RUNNING;
    while (swing_exit == RUNNING) {
      swing_exit = swing_exit != RUNNING ? swing_exit : swingPID.exit_condition({drive_motors});
      pros::delay(DELAY_TIME);
    }
    std::cout << "  Swing: " << exit_to_string(swing_exit) << " Exit.\n";

    // if (a_exit == mA_EXIT || a_exit == VELOCITY_EXIT) {
    //   interfered = true;
    // }
  }
}

void pp_wait_until(int index) {
  // Let the PID run at least 1 iteration
  pros::delay(DELAY_TIME);

  while (pp_index < injected_pp_index[index - 1]) {
    pros::delay(DELAY_TIME);
  }
}