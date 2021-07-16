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

#include "four_wheel_steering_controller/model.hpp"
#include <limits>

namespace four_wheel_steering_controller
{

// see https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template<typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

using four_wheel_steering_msgs::msg::FourWheelSteering;

GenericFourWheelSteering::GenericFourWheelSteering(VehicleParams vehicle_params)
: vehicle_params_(vehicle_params)
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

SteeringTurn GenericFourWheelSteering::compute_turn(
  double tan_front_steering_angle,
  double tan_rear_steering_angle) const
{
  SteeringTurn turn;
  const auto front_angle_ratio = tan_front_steering_angle /
    (tan_front_steering_angle - tan_rear_steering_angle);
  const auto rear_angle_ratio = tan_front_steering_angle /
    (tan_front_steering_angle - tan_rear_steering_angle);

  const auto front_distance_to_irc = vehicle_params_.wheel_base * front_angle_ratio;
  const auto rear_distance_to_irc = vehicle_params_.wheel_base * rear_angle_ratio;

  const auto lateral_distance_to_irc = front_distance_to_irc / tan_front_steering_angle;

  const auto half_wheel_base = vehicle_params_.wheel_base / 2.0;

  if (sgn(tan_front_steering_angle) == sgn(tan_rear_steering_angle)) {
    // positive 4ws
    turn.turn_radius =
      std::hypot(half_wheel_base + rear_distance_to_irc, lateral_distance_to_irc);
  } else {
    // negative 4ws
    turn.turn_radius =
      std::hypot(half_wheel_base + rear_distance_to_irc, lateral_distance_to_irc);
  }

  double inner_lateral_distance_to_irc = lateral_distance_to_irc - steering_track_ / 2.0;
  double outer_lateral_distance_to_irc = lateral_distance_to_irc + steering_track_ / 2.0;

  double front_left_lateral_distance;
  double front_right_lateral_distance;
  double rear_left_lateral_distance;
  double rear_right_lateral_distance;
  if (tan_front_steering_angle >= 0.0) {
    // left turn
    front_left_lateral_distance = inner_lateral_distance_to_irc;
    rear_left_lateral_distance = inner_lateral_distance_to_irc;
    front_right_lateral_distance = outer_lateral_distance_to_irc;
    rear_right_lateral_distance = outer_lateral_distance_to_irc;
  } else {
    // right turn
    front_left_lateral_distance = outer_lateral_distance_to_irc;
    rear_left_lateral_distance = outer_lateral_distance_to_irc;
    front_right_lateral_distance = inner_lateral_distance_to_irc;
    rear_right_lateral_distance = inner_lateral_distance_to_irc;
  }

  turn.front_left_turn_radius =
    std::hypot(front_distance_to_irc, front_left_lateral_distance);
  turn.front_right_turn_radius = std::hypot(
    front_distance_to_irc,
    front_right_lateral_distance);
  turn.rear_left_turn_radius = std::hypot(rear_distance_to_irc, rear_left_lateral_distance);
  turn.rear_right_turn_radius =
    std::hypot(rear_distance_to_irc, rear_right_lateral_distance);

  // consider offset of steering joint to wheel in computing turn radius
  if (tan_front_steering_angle >= 0.0) {
    // left turn
    turn.front_left_turn_radius -= vehicle_params_.distance_steering_to_wheel;
    turn.front_right_turn_radius += vehicle_params_.distance_steering_to_wheel;
    turn.rear_left_turn_radius -= vehicle_params_.distance_steering_to_wheel;
    turn.rear_right_turn_radius += vehicle_params_.distance_steering_to_wheel;
  } else {
    // right turn
    turn.front_left_turn_radius += vehicle_params_.distance_steering_to_wheel;
    turn.front_right_turn_radius -= vehicle_params_.distance_steering_to_wheel;
    turn.rear_left_turn_radius += vehicle_params_.distance_steering_to_wheel;
    turn.rear_right_turn_radius -= vehicle_params_.distance_steering_to_wheel;
  }
  return turn;
}

FourWheelSteering GenericFourWheelSteering::compute_odometry(
  const FourWheelSteeringState & state) const
{
  rcpputils::require_true(
    fabs(
      state.front_left_steering_angle) < M_PI_4, "steering angle must be within [-pi/4, pi/4]");

  FourWheelSteering result;
  auto front_rear_delta =
    std::abs(state.front_left_steering_angle - state.rear_left_steering_angle);
  if ((std::abs(state.front_left_steering_angle) >
    STRAIGHT_DRIVING_THRESHOLD || std::abs(state.rear_left_steering_angle) >
    STRAIGHT_DRIVING_THRESHOLD) && front_rear_delta > STRAIGHT_DRIVING_THRESHOLD)
  {
    const auto tan_front_left_steering_angle = std::tan(state.front_left_steering_angle);
    const auto tan_front_right_steering_angle = std::tan(state.front_right_steering_angle);
    const auto tan_rear_left_steering_angle = std::tan(state.rear_left_steering_angle);
    const auto tan_rear_right_steering_angle = std::tan(state.rear_right_steering_angle);

    const auto tan_front_steering_angle = 2 * tan_front_left_steering_angle *
      tan_front_right_steering_angle /
      (tan_front_left_steering_angle + tan_front_right_steering_angle);

    const auto tan_rear_steering_angle = 2 * tan_rear_left_steering_angle *
      tan_rear_right_steering_angle /
      (tan_rear_left_steering_angle + tan_rear_right_steering_angle);

    const auto turn = compute_turn(tan_front_steering_angle, tan_rear_steering_angle);

    const auto front_left_vel = state.front_left_motor_radial_vel * turn.front_left_turn_radius /
      turn.turn_radius;
    const auto front_right_vel = state.front_right_motor_radial_vel * turn.front_right_turn_radius /
      turn.turn_radius;
    const auto rear_left_vel = state.rear_left_motor_radial_vel * turn.rear_left_turn_radius /
      turn.turn_radius;
    const auto rear_right_vel = state.rear_right_motor_radial_vel * turn.rear_right_turn_radius /
      turn.turn_radius;

    const auto radial_velocity =
      (front_left_vel + front_right_vel + rear_left_vel + rear_right_vel) /
      4.0;

    result.front_steering_angle = std::atan(tan_front_steering_angle);
    result.rear_steering_angle = std::atan(tan_rear_steering_angle);
    result.speed = radial_velocity * vehicle_params_.wheel_radius;
  } else {
    const auto radial_velocity =
      (state.front_left_motor_radial_vel + state.front_right_motor_radial_vel +
      state.rear_left_motor_radial_vel + state.rear_right_motor_radial_vel) /
      4.0;

    result.front_steering_angle =
      (state.front_left_steering_angle + state.front_right_steering_angle) / 2.0;
    result.rear_steering_angle =
      (state.rear_left_steering_angle + state.rear_right_steering_angle) / 2.0;
    result.speed = radial_velocity * vehicle_params_.wheel_radius;
  }
  return result;
}

FourWheelSteeringCommands GenericFourWheelSteering::compute_commands(
  const FourWheelSteering & cmd_4ws) const
{
  FourWheelSteeringCommands cmd;
  double front_left_speed;
  double front_right_speed;
  double rear_left_speed;
  double rear_right_speed;

  auto front_rear_delta =
    std::abs(cmd_4ws.front_steering_angle - cmd_4ws.rear_steering_angle);
  if ((std::abs(cmd_4ws.front_steering_angle) >
    STRAIGHT_DRIVING_THRESHOLD || std::abs(cmd_4ws.rear_steering_angle) >
    STRAIGHT_DRIVING_THRESHOLD) && front_rear_delta > STRAIGHT_DRIVING_THRESHOLD)
  {
    const auto tan_front_steering_angle = std::tan(cmd_4ws.front_steering_angle);
    const auto tan_rear_steering_angle = std::tan(cmd_4ws.rear_steering_angle);
    const auto turn = compute_turn(tan_front_steering_angle, tan_rear_steering_angle);

    front_left_speed = cmd_4ws.speed * turn.front_left_turn_radius / turn.turn_radius;
    front_right_speed = cmd_4ws.speed * turn.front_right_turn_radius / turn.turn_radius;
    rear_left_speed = cmd_4ws.speed * turn.rear_left_turn_radius / turn.turn_radius;
    rear_right_speed = cmd_4ws.speed * turn.rear_right_turn_radius / turn.turn_radius;
  } else {
    // in straight driving, apply all steering angles and velocities equally
    front_left_speed = cmd_4ws.speed;
    front_right_speed = cmd_4ws.speed;
    rear_left_speed = cmd_4ws.speed;
    rear_right_speed = cmd_4ws.speed;
  }
  cmd.front_steering_motor_angle = cmd_4ws.front_steering_angle *
    vehicle_params_.steering_gear_transmission_ratio;
  cmd.rear_steering_motor_angle = cmd_4ws.rear_steering_angle *
    vehicle_params_.steering_gear_transmission_ratio;

  cmd.front_left_motor_radial_vel = front_left_speed / vehicle_params_.wheel_radius;
  cmd.front_right_motor_radial_vel = front_right_speed / vehicle_params_.wheel_radius;
  cmd.rear_left_motor_radial_vel = rear_left_speed / vehicle_params_.wheel_radius;
  cmd.rear_right_motor_radial_vel = rear_right_speed / vehicle_params_.wheel_radius;
  return cmd;
}

}  // namespace four_wheel_steering_controller
