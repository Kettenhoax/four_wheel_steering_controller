// Copyright 2021 Austrian Institute of Technology GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rcpputils/asserts.hpp>
#include <cmath>
#include <four_wheel_steering_msgs/msg/four_wheel_steering.hpp>

namespace four_wheel_steering_controller
{

using four_wheel_steering_msgs::msg::FourWheelSteering;

struct VehicleParams
{
  double wheel_base = 0.0;
  double wheel_track = 0.0;
  double wheel_radius = 0.0;
  double distance_steering_to_wheel = 0.0;
  double steering_gear_transmission_ratio = 1.0;
};

struct FourWheelSteeringState
{
  double front_left_motor_radial_vel;
  double front_right_motor_radial_vel;
  double rear_left_motor_radial_vel;
  double rear_right_motor_radial_vel;
  double front_left_steering_angle;
  double front_right_steering_angle;
  double rear_left_steering_angle;
  double rear_right_steering_angle;
};

struct FourWheelSteeringCommands
{
  double front_left_motor_radial_vel;
  double front_right_motor_radial_vel;
  double rear_left_motor_radial_vel;
  double rear_right_motor_radial_vel;
  double front_steering_motor_angle;
  double rear_steering_motor_angle;
};

class SteeringModel
{
public:
  virtual FourWheelSteering compute_odometry(const FourWheelSteeringState & state) const = 0;

  virtual FourWheelSteeringCommands compute_commands(const FourWheelSteering & cmd_4ws) const = 0;
};

struct SteeringTurn
{
  double turn_radius;
  double front_left_turn_radius;
  double front_right_turn_radius;
  double rear_left_turn_radius;
  double rear_right_turn_radius;
};

class GenericFourWheelSteering : public SteeringModel
{
private:
  VehicleParams vehicle_params_;
  double steering_track_;

  // minimum steering angle difference to consider the vehicle to take a turn
  static constexpr double STRAIGHT_DRIVING_THRESHOLD = 1e-2;

  SteeringTurn compute_turn(double tan_front_steering_angle, double tan_rear_steering_angle) const;

public:
  explicit GenericFourWheelSteering(VehicleParams vehicle_params);

  virtual FourWheelSteering compute_odometry(const FourWheelSteeringState & state) const;

  virtual FourWheelSteeringCommands compute_commands(const FourWheelSteering & cmd_4ws) const;
};

}  // namespace four_wheel_steering_controller
