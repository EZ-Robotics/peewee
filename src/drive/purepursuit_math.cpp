/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

// Conversions from deg to rad and rad to deg
double to_deg(double input) { return input * (180 / M_PI); }
double to_rad(double input) { return input * (M_PI / 180); }

// Outputs angle within 180 t0 -180
double wrap_angle(double theta) {
  while (theta > 180) theta -= 360;
  while (theta < -180) theta += 360;
  return theta;
}

// Finds error in shortest angle to point
double absolute_angle_to_point(pose itarget, pose icurrent) {
  // Difference in target to current (legs of triangle)
  double x_error = itarget.x - icurrent.x;
  double y_error = itarget.y - icurrent.y;

  // Displacement of error
  double error = to_deg(atan2(x_error, y_error));
  return error;
}

// Finds error between target and current
double relative_angle_to_point(double angle) {
  return wrap_angle(angle - get_angle());
}

// Find shortest distance to point
double distance_to_point(pose itarget, pose icurrent) {
  // Difference in target to current (legs of triangle)
  double x_error = (itarget.x - icurrent.x);
  double y_error = (itarget.y - icurrent.y);

  // Hypotenuse of triangle
  double distance = hypot(x_error, y_error);

  return distance;
}

// Uses input as hypot to find the new xy
pose vector_off_point(double added, pose icurrent) {
  double x_error = sin(to_rad(icurrent.theta)) * added;
  double y_error = cos(to_rad(icurrent.theta)) * added;

  pose output;
  output.x = x_error + icurrent.x;
  output.y = y_error + icurrent.y;
  output.theta = icurrent.theta;
  return output;
}

// Creates carrot point so robot ends at desired angle
// based on https://www.desmos.com/calculator/sptjw5szex
double dlead = 1;
std::vector<pose> boomerang(pose start, pose itarget) {
  double h = std::hypot(start.x - itarget.x, start.y - itarget.y);
  std::vector<pose> carrot;
  double new_x = itarget.x - h * sin(to_rad(itarget.theta)) * dlead;
  double new_y = itarget.y - h * cos(to_rad(itarget.theta)) * dlead;
  carrot.push_back({new_x, new_y});
  carrot.push_back({itarget.x, itarget.y});
  return carrot;
}

// Inject point based on https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552
std::vector<odom> inject_points(std::vector<odom> imovements) {
  injected_pp_index.clear();
  // Create new vector that includes the starting point
  std::vector<odom> input = imovements;
  input.insert(input.begin(), {{{target.x, target.y, imovements[0].target.theta}, imovements[0].turn_type, imovements[0].max_xy_speed}});

  std::vector<odom> output;  // Output vector
  int output_index = -1;     // Keeps track of current index

  // This for loop runs for how many points there are minus one because there is one less vector then points
  for (int i = 0; i < input.size() - 1; i++) {
    // Figure out how many points fit in the vector
    int num_of_points_that_fit = (distance_to_point(input[i + 1].target, input[i].target)) / SPACING;

    // Add parent point
    // Make sure the robot is looking at next point
    output.push_back({input[i].target,
                      input[i + 1].turn_type,
                      input[i].max_xy_speed});
    output_index++;
    injected_pp_index.push_back(output_index);

    // Add the injected points
    for (int j = 0; j < num_of_points_that_fit; j++) {
      // Calculate the new point with known information: hypot and angle
      double angle_to_point = absolute_angle_to_point(input[i + 1].target, input[i].target);
      pose new_point = vector_off_point(SPACING, {output[output_index].target.x, output[output_index].target.y, angle_to_point});

      // Make sure the robot is looking at next point
      turn_types turn;
      turn = input[i + 1].turn_type;

      // Push new point to vector
      output.push_back({{new_point.x, new_point.y, input[i + 1].target.theta},
                        turn,
                        input[i + 1].max_xy_speed});
      output_index++;
    }
  }

  // Update output angles
  // output = update_path_angles(output);

  // Return final vector
  return output;
}

// Path smoothing based on https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4
std::vector<odom> smooth_path(std::vector<odom> ipath, double weight_smooth, double weight_data, double tolerance) {
  double path[1000][2];
  double new_path[1000][2];

  // Convert odom to array
  for (int i = 0; i < ipath.size(); i++) {
    path[i][0] = new_path[i][0] = ipath[i].target.x;
    path[i][1] = new_path[i][1] = ipath[i].target.y;
  }

  double change = tolerance;

  while (change >= tolerance) {
    change = 0.0;
    for (int i = 1; i < ipath.size() - 2; i++) {
      for (int j = 0; j < 2; j++) {
        double x_i = path[i][j];
        double y_i = new_path[i][j];
        double y_prev = new_path[i - 1][j];
        double y_next = new_path[i + 1][j];

        double y_i_saved = y_i;
        y_i += weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2.0 * y_i));
        new_path[i][j] = y_i;

        change += abs(y_i - y_i_saved);
      }
    }
  }

  // Convert array to odom
  std::vector<odom> output = ipath;  // Set output to input so target angles, turn types and speed hold
  // Overwrite x and y
  for (int i = 0; i < ipath.size(); i++) {
    output[i].target.x = new_path[i][0];
    output[i].target.y = new_path[i][1];
  }

  // Update output angles
  // output = update_path_angles(output);

  // Return final vector
  return output;
}

// Calculates what target angle should be
std::vector<odom> update_path_angles(std::vector<odom> ipath) {
  double angle = 0;
  odom temp;
  std::vector<odom> output;

  for (int i = 0; i < ipath.size(); i++) {
    // If checking last point, make angle the same as last
    // Otherwise, calculate angle between point0 and point1
    if (i == ipath.size() - 1)
      angle = output[i - 1].target.theta;
    else
      angle = absolute_angle_to_point(ipath[i + 1].target, ipath[i].target);  // Calculate what the angle should be

    // Update angle
    temp = ipath[i];
    temp.target.theta = angle;
    output.push_back(temp);
  }

  // Return new path
  return output;
}

// Calculates arc to follow to get robot to desired end angle
// made and debugged with ChatGPT
std::vector<odom> pointsAlongArc(const pose startingPoint, double endingAngle, turn_types direction, int max_speed, double radius) {
  std::vector<odom> output;
  double startingAngle = to_rad(startingPoint.theta);
  endingAngle = to_rad(endingAngle);

  // Calculate the arc length
  double arcLength = radius * std::abs(endingAngle - startingAngle);

  // Calculate the number of points based on the desired spacing
  int numPoints = std::max(2, static_cast<int>(std::round(arcLength / SPACING)) + 1);

  // Calculate the center of the circle
  pose center;
  center = vector_off_point(radius, {startingPoint.x, startingPoint.y, startingPoint.theta - to_deg(endingAngle)});

  // Calculate the increment for each point
  double increment = (endingAngle > startingAngle ? 1.0 : -1.0) * std::abs(endingAngle - startingAngle) / (numPoints - 1);

  // Loop through the points and calculate their positions
  for (int i = 0; i < numPoints; i++) {
    // Calculate the angle for this point
    double angle = startingAngle + increment * i;

    // Calculate the position of the point along the arc
    double x = center.x + radius * cos(angle);
    double y = center.y + radius * sin(angle);

    pose tempxy;
    tempxy.x = x;
    tempxy.y = y;

    odom temp;
    temp.target = tempxy;
    temp.max_xy_speed = max_speed;
    temp.turn_type = direction;

    // Add the point to a list of points
    output.push_back(temp);
  }

  // Update output angles
  // update_path_angles(output);

  printf("center (%.2f, %.2f)\n", center.x, center.y);
  for (int i = 0; i < output.size(); i++) {
    printf("%i:  (%.2f, %.2f)\n", i + 1, output[i].target.x, output[i].target.y);
  }
  printf("\n");

  return output;
}
