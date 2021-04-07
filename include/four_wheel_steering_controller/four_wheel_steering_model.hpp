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
#include <limits>
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
  double front_left_steering_angle;
  double front_right_steering_angle;
  double rear_left_steering_angle;
  double rear_right_steering_angle;
  double front_steering_motor_angle;
  double rear_steering_motor_angle;
};

class SymmetricNegativeFourWheelSteering
{
private:
  VehicleParams vehicle_params_;
  bool verify_;
  double steering_track_;

  // minimum steering angle in radians to consider the vehicle to take a turn
  static constexpr double TURN_ANGLE_THRESHOLD = 1e-5;

  // turn radii derived from inner and outer steering angle must match by this margin
  static constexpr double TURN_RADIUS_VERIFICATION_MARGIN = 1e-3;

  // radial velocities derived from inner and outer wheel must match by this margin
  static constexpr double RADIAL_VEL_VERIFICATION_MARGIN = 1e-3;

  // compute turn radius of inner wheel
  double compute_inner_turn_radius(double turn_radius, double inner_steering_angle) const;

  // compute turn radius of outer wheel
  double compute_outer_turn_radius(double turn_radius, double outer_steering_angle) const;

public:
// verify enables checking whether the measured state is actually symmetric,
// average checks whether the inputs are symmetric AND uses a mean of inputs to compute the output
  explicit SymmetricNegativeFourWheelSteering(
    VehicleParams vehicle_params, bool verify = true)
  : vehicle_params_(vehicle_params), verify_(verify)
  {
    rcpputils::require_true(
      vehicle_params_.wheel_base >= std::numeric_limits<double>::epsilon(),
      "wheel_base must be a non-zero positive real number");
    rcpputils::require_true(
      vehicle_params_.wheel_track >= std::numeric_limits<double>::epsilon(),
      "wheel_track must be a positive real number");
    rcpputils::require_true(
      vehicle_params_.wheel_radius >= std::numeric_limits<double>::epsilon(),
      "wheel_radius must be a positive real number");
    rcpputils::require_true(
      vehicle_params_.distance_steering_to_wheel >= 0.0,
      "distance_steering_to_wheel must be a positive real number");
    steering_track_ = vehicle_params_.wheel_track - 2 * vehicle_params_.distance_steering_to_wheel;
  }

  FourWheelSteering compute_odometry(const FourWheelSteeringState & state) const;

  FourWheelSteeringCommands compute_commands(const FourWheelSteering & cmd_4ws) const;
};

double SymmetricNegativeFourWheelSteering::compute_inner_turn_radius(
  double turn_radius,
  double inner_steering_angle)
const
{
  const double half_wheelbase = vehicle_params_.wheel_base / 2.0;
  const double half_track = steering_track_ / 2.0;
  const double d = vehicle_params_.distance_steering_to_wheel;
  const double inner_wheel_x = half_wheelbase - fabs(std::sin(inner_steering_angle)) * d;
  const double inner_wheel_y = turn_radius -
    (half_track + std::cos(inner_steering_angle) * d);
  return std::hypot(inner_wheel_x, inner_wheel_y);
}

double SymmetricNegativeFourWheelSteering::compute_outer_turn_radius(
  double turn_radius,
  double outer_steering_angle)
const
{
  const double half_wheelbase = vehicle_params_.wheel_base / 2.0;
  const double half_track = steering_track_ / 2.0;
  const double d = vehicle_params_.distance_steering_to_wheel;
  const double outer_wheel_x = half_wheelbase + fabs(std::sin(outer_steering_angle)) * d;
  const double outer_wheel_y = turn_radius +
    (half_track + std::cos(outer_steering_angle) * d);
  return std::hypot(outer_wheel_x, outer_wheel_y);
}

FourWheelSteering SymmetricNegativeFourWheelSteering::compute_odometry(
  const FourWheelSteeringState & state) const
{
  rcpputils::require_true(
    fabs(
      state.front_left_steering_angle) < M_PI_4, "steering angle must be within [-pi/4, pi/4]");

  FourWheelSteering result;
  double radial_vel = 0.0;
  if (fabs(state.front_left_steering_angle) > TURN_ANGLE_THRESHOLD) {
    // in steered driving, compute central velocity and steering angle
    const double half_wheelbase = vehicle_params_.wheel_base / 2.0;
    const double half_track = steering_track_ / 2.0;
    const double inner_steering_angle = state.front_left_steering_angle >
      0.0 ? state.front_left_steering_angle : -state.front_right_steering_angle;

    const double outer_steering_angle = state.front_left_steering_angle <
      0.0 ? -state.front_left_steering_angle : state.front_right_steering_angle;

    double turn_radius = half_wheelbase / std::tan(inner_steering_angle) + half_track;
    if (verify_) {
      const double turn_radius_by_outer = half_wheelbase / std::tan(outer_steering_angle) -
        half_track;
      rcpputils::check_true(
        fabs(turn_radius_by_outer) - fabs(turn_radius) < TURN_RADIUS_VERIFICATION_MARGIN,
        "turn radius according to outer steering angle mismatches with inner steering angle");
      turn_radius = (turn_radius + turn_radius_by_outer) / 2.0;
    }

    const double inner_radial_vel = state.front_left_steering_angle >
      0.0 ? state.front_left_motor_radial_vel : state.front_right_motor_radial_vel;
    const double inner_turn_radius = compute_inner_turn_radius(turn_radius, inner_steering_angle);

    // project inner wheel velocity to velocity at center of vehicle
    radial_vel = turn_radius / inner_turn_radius * inner_radial_vel;

    if (verify_) {
      const double outer_turn_radius = compute_outer_turn_radius(turn_radius, outer_steering_angle);
      const double outer_radial_vel = state.front_left_steering_angle <
        0.0 ? state.front_left_motor_radial_vel : state.front_right_motor_radial_vel;
      double radial_vel_by_outer_wheel = turn_radius / outer_turn_radius * outer_radial_vel;
      rcpputils::check_true(
        fabs(radial_vel_by_outer_wheel) - fabs(radial_vel) < RADIAL_VEL_VERIFICATION_MARGIN,
        "radial velocity according to outer wheel mismatches with inner wheel");
      radial_vel = (radial_vel + radial_vel_by_outer_wheel) / 2.0;
    }

    const double steering_angle = atan(half_wheelbase / turn_radius);
    const double sign = copysign(1.0, state.front_left_steering_angle);
    result.front_steering_angle = sign * steering_angle;
    result.rear_steering_angle = sign * -steering_angle;
  } else {
    // in straight driving, assume equal velocities and steering angles
    radial_vel = state.front_left_motor_radial_vel +
      state.front_right_motor_radial_vel + state.rear_left_motor_radial_vel +
      state.rear_right_motor_radial_vel;
    radial_vel /= 4.0;
    result.front_steering_angle = state.front_left_steering_angle;
    result.rear_steering_angle = state.front_left_steering_angle;
  }
  result.speed = radial_vel * vehicle_params_.wheel_radius;
  return result;
}

FourWheelSteeringCommands SymmetricNegativeFourWheelSteering::compute_commands(
  const FourWheelSteering & cmd_4ws) const
{
  rcpputils::require_true(
    cmd_4ws.front_steering_angle + cmd_4ws.rear_steering_angle <
    std::numeric_limits<double>::epsilon(),
    "To use the symmetric negative model front and rear steering angle must add up to 0");

  FourWheelSteeringCommands cmd;
  double front_left_speed = 0.0;
  double front_right_speed = 0.0;
  double rear_left_speed = 0.0;
  double rear_right_speed = 0.0;

  const double half_wheelbase = vehicle_params_.wheel_base / 2.0;
  const double half_track = steering_track_ / 2.0;

  if (fabs(cmd_4ws.front_steering_angle) > TURN_ANGLE_THRESHOLD) {
    // in steered driving compute optimal angles and velocities per-wheel
    const double turn_radius = fabs(half_wheelbase / std::tan(cmd_4ws.front_steering_angle));

    const double inner_steering_angle = std::atan(half_wheelbase / (turn_radius - half_track));
    const double outer_steering_angle = std::atan(half_wheelbase / (turn_radius + half_track));

    const double inner_turn_radius = compute_inner_turn_radius(turn_radius, inner_steering_angle);
    const double outer_turn_radius = compute_outer_turn_radius(turn_radius, inner_steering_angle);

    const double inner_wheel_linear_vel = inner_turn_radius / turn_radius * cmd_4ws.speed;
    const double outer_wheel_linear_vel = outer_turn_radius / turn_radius * cmd_4ws.speed;

    if (cmd_4ws.front_steering_angle > 0.0) {
      // left turning
      cmd.front_left_steering_angle = inner_steering_angle;
      cmd.front_right_steering_angle = outer_steering_angle;
      cmd.rear_left_steering_angle = -inner_steering_angle;
      cmd.rear_right_steering_angle = -outer_steering_angle;

      front_left_speed = inner_wheel_linear_vel;
      front_right_speed = outer_wheel_linear_vel;
      rear_left_speed = inner_wheel_linear_vel;
      rear_right_speed = outer_wheel_linear_vel;
    } else {
      // right turning
      cmd.front_left_steering_angle = -outer_steering_angle;
      cmd.front_right_steering_angle = -inner_steering_angle;
      cmd.rear_left_steering_angle = outer_steering_angle;
      cmd.rear_right_steering_angle = inner_steering_angle;

      front_left_speed = outer_wheel_linear_vel;
      front_right_speed = inner_wheel_linear_vel;
      rear_left_speed = outer_wheel_linear_vel;
      rear_right_speed = inner_wheel_linear_vel;
    }
  } else {
    // in straight driving, apply all steering angles and velocities equally
    cmd.front_left_steering_angle = cmd_4ws.front_steering_angle;
    cmd.front_right_steering_angle = cmd_4ws.front_steering_angle;
    cmd.rear_left_steering_angle = cmd_4ws.rear_steering_angle;
    cmd.rear_right_steering_angle = cmd_4ws.rear_steering_angle;

    front_left_speed = cmd_4ws.speed;
    front_right_speed = cmd_4ws.speed;
    rear_left_speed = cmd_4ws.speed;
    rear_right_speed = cmd_4ws.speed;
  }
  cmd.front_steering_motor_angle = cmd_4ws.front_steering_angle *
    -vehicle_params_.steering_gear_transmission_ratio;
  cmd.rear_steering_motor_angle = cmd_4ws.rear_steering_angle *
    -vehicle_params_.steering_gear_transmission_ratio;

  cmd.front_left_motor_radial_vel = front_left_speed / vehicle_params_.wheel_radius;
  cmd.front_right_motor_radial_vel = front_right_speed / vehicle_params_.wheel_radius;
  cmd.rear_left_motor_radial_vel = rear_left_speed / vehicle_params_.wheel_radius;
  cmd.rear_right_motor_radial_vel = rear_right_speed / vehicle_params_.wheel_radius;
  return cmd;
}

}  // namespace four_wheel_steering_controller
