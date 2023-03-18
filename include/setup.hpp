/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "api.h"
#include "pros/adi.hpp"

/**
 * Auton Tuned Default Speeds
 */
extern const int MAX_XY;
extern const int TURN_SPEED;

extern const double STOP_UPDATING_ANGLE;  // When looking at a target, stop updating the angle when target is within this
extern const double LOOK_AHEAD;           // Pure pursuit look ahead distance
extern const double SPACING;              // Spacing for pure pursuit injected points

/**
 * Ports
 */
inline pros::Motor l1(11);
inline pros::Motor r1(-20);

inline pros::Imu imu(5);

inline const std::vector<pros::Motor> left_motors = {l1};
inline const std::vector<pros::Motor> right_motors = {r1};
inline const std::vector<pros::Motor> drive_motors = {l1, r1};

// inline pros::ADIEncoder center_tracker('C', 'D', true);
// inline pros::ADIEncoder left_tracker('A', 'B', true);
// inline pros::ADIEncoder right_tracker('C', 'D');

/**
 * Tracking Wheel Constants
 */
// Wheel size and encoder
extern const int CART;
extern const double RATIO;
extern const int TICK_PER_REV;
extern const double WHEEL_DIA;

inline const double TICK_PER_INCH = (TICK_PER_REV / (WHEEL_DIA * M_PI));

// Tracking wheel offsets
extern const double WIDTH;
extern const double IMU_SCALER;
// inline const double CENTER_OFFSET = -2.0;

// ignore these unless the left/right tracker aren't mounted symmetrically
extern const double RIGHT_OFFSET;
extern const double LEFT_OFFSET;
