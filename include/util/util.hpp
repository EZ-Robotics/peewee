/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include <bits/stdc++.h>
#include <stdio.h>
#include <string.h>

#include "api.h"
#include "setup.hpp"

inline pros::Controller master(pros::E_CONTROLLER_MASTER);

/**
 * Enum for drive types.
 */
enum e_mode { DISABLE = 0,
              DISABLED = 0,
              SWING = 1,
              SWING_TO_POINT = 2,
              TURN = 3,
              TURN_TO_POINT = 4,
              DRIVE = 5,
              TO_POINT = 6,
              PURE_PURSUIT = 7 };

/**
 * Enum for exit_condition outputs.
 */
enum exit_output { RUNNING = 1,
                   SMALL_EXIT = 2,
                   BIG_EXIT = 3,
                   VELOCITY_EXIT = 4,
                   mA_EXIT = 5,
                   ERROR_NO_CONSTANTS = 6 };

/**
 * Enum for turn types
 */
enum turn_types { FWD = 0,
                  REV = 1,
};

/**
 * Enum for swing types
 */
enum swing_types { LEFT_SWING = 0,
                   L_SWING = 0,
                   L_S = 0,
                   LS = 0,
                   RIGHT_SWING = 1,
                   R_SWING = 1,
                   R_S = 1,
                   RS = 1
};

/**
 * Struct for coordinates
 */
typedef struct pose {
  double x;
  double y;
  double theta;
} pose;

/**
 * Struct for odom movements
 */
typedef struct odom {
  pose target;
  turn_types turn_type;
  int max_xy_speed = MAX_XY;
} odom;

/**
 * Outputs string for exit_condition enum.
 */
std::string exit_to_string(exit_output input);

/**
 * Outputs string for turn_types enum.
 */
std::string turn_types_to_string(turn_types input);

const int DELAY_TIME = 10;

inline int mode = DISABLE;
inline bool AUTO_RAN = false;

/**
 * Returns input restricted to min-max threshold
 */
double clip_num(double input, double max, double min);

/**
 * Returns 1 if input is positive and -1 if input is negative
 */
int sgn(double input);

/**
 * Convert radians to degrees
 */
double to_deg(double input);

/**
 * Convert degrees to radians
 */
double to_rad(double input);

void print_path_for_python(std::vector<odom> imovements);

/**
 * Is the SD card plugged in?
 */
const bool IS_SD_CARD = pros::usd::is_installed();