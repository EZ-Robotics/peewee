/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once
#include "api.h"

inline pros::Motor intake(7);
inline const std::vector<pros::Motor> intake_motors = {intake};

void set_intake(int input);

void intake_opcontrol();