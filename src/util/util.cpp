/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <string.h>

#include "main.h"

bool AUTON_RAN = true;

double clip_num(double input, double max, double min) {
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}

int sgn(double input) {
  if (input > 0)
    return 1;
  else if (input < 0)
    return -1;
  return 0;
}

// Print exit conditions
std::string exit_to_string(exit_output input) {
  switch ((int)input) {
    case RUNNING:
      return "Running";
    case SMALL_EXIT:
      return "Small";
    case BIG_EXIT:
      return "Big";
    case VELOCITY_EXIT:
      return "Velocity";
    case mA_EXIT:
      return "mA";
    case ERROR_NO_CONSTANTS:
      return "Error: Exit condition constants not set!";
    default:
      return "Error: Out of bounds!";
  }

  return "Error: Out of bounds!";
}

// Print turn types
std::string turn_types_to_string(turn_types input) {
  switch ((int)input) {
    case FWD:
      return "FWD";
    case REV:
      return "REV";
    default:
      return "Error: Out of bounds!";
  }

  return "Error: Out of bounds!";
}
void print_path_for_python(std::vector<odom> imovements) {
  bool first = true;
  // Print subpoints
  std::cout << "raw_path = [";
  for (int i = 0; i < imovements.size(); i++) {
    if (!first) std::cout << "   ,";

    std::cout << "[" << imovements[i].target.x << ", " << imovements[i].target.y << ", " << imovements[i].target.theta << ", \"" << turn_types_to_string(imovements[i].turn_type) << "\", " << imovements[i].max_xy_speed << "] \n";
    // std::cout << "[" << imovements[i].target.x / 12.0 << ", " << imovements[i].target.y / 12.0 << "] \n";

    first = false;
  }
  std::cout << "]\n";
}