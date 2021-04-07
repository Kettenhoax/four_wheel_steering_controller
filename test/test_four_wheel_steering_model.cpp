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
#include "four_wheel_steering_controller/four_wheel_steering_model.hpp"
#include "four_wheel_steering_msgs/msg/four_wheel_steering.hpp"

#include <gmock/gmock.h>
#include <cmath>
#include <memory>
#include <limits>

using four_wheel_steering_msgs::msg::FourWheelSteering;
using four_wheel_steering_controller::VehicleParams;
using four_wheel_steering_controller::FourWheelSteeringState;
using four_wheel_steering_controller::SymmetricNegativeFourWheelSteering;

class TestFourWheelSteeringModel : public ::testing::Test
{
protected:
  std::unique_ptr<SymmetricNegativeFourWheelSteering> model_;

  void SetUp() override
  {
    VehicleParams params;
    params.wheel_base = 1.5;
    params.wheel_track = 1.0;
    params.wheel_radius = 0.25;
    params.distance_steering_to_wheel = 0.0;
    params.steering_gear_transmission_ratio = 1.0;
    model_ = std::make_unique<SymmetricNegativeFourWheelSteering>(params);
  }
};

TEST_F(TestFourWheelSteeringModel, test_straight_drive)
{
  constexpr double THRESHOLD = std::numeric_limits<double>::epsilon();

  FourWheelSteering cmd_4ws;
  cmd_4ws.speed = 1.0;
  auto cmds = model_->compute_commands(cmd_4ws);

  EXPECT_NEAR(cmds.front_left_motor_radial_vel, 4.0, THRESHOLD);
  EXPECT_NEAR(cmds.front_right_motor_radial_vel, 4.0, THRESHOLD);
  EXPECT_NEAR(cmds.rear_left_motor_radial_vel, 4.0, THRESHOLD);
  EXPECT_NEAR(cmds.rear_right_motor_radial_vel, 4.0, THRESHOLD);

  EXPECT_NEAR(cmds.front_left_steering_angle, 0.0, THRESHOLD);
  EXPECT_NEAR(cmds.front_right_steering_angle, 0.0, THRESHOLD);
  EXPECT_NEAR(cmds.rear_left_steering_angle, 0.0, THRESHOLD);
  EXPECT_NEAR(cmds.rear_right_steering_angle, 0.0, THRESHOLD);

  EXPECT_NEAR(cmds.front_steering_motor_angle, 0.0, THRESHOLD);
  EXPECT_NEAR(cmds.rear_steering_motor_angle, 0.0, THRESHOLD);
}

TEST_F(TestFourWheelSteeringModel, test_reverse_drive)
{
  constexpr double THRESHOLD = std::numeric_limits<double>::epsilon();

  FourWheelSteering cmd_4ws;
  cmd_4ws.speed = -1.0;
  auto cmds = model_->compute_commands(cmd_4ws);

  EXPECT_NEAR(cmds.front_left_motor_radial_vel, -4.0, THRESHOLD);
  EXPECT_NEAR(cmds.front_right_motor_radial_vel, -4.0, THRESHOLD);
  EXPECT_NEAR(cmds.rear_left_motor_radial_vel, -4.0, THRESHOLD);
  EXPECT_NEAR(cmds.rear_right_motor_radial_vel, -4.0, THRESHOLD);

  EXPECT_NEAR(cmds.front_left_steering_angle, 0.0, THRESHOLD);
  EXPECT_NEAR(cmds.front_right_steering_angle, 0.0, THRESHOLD);
  EXPECT_NEAR(cmds.rear_left_steering_angle, 0.0, THRESHOLD);
  EXPECT_NEAR(cmds.rear_right_steering_angle, 0.0, THRESHOLD);

  EXPECT_NEAR(cmds.front_steering_motor_angle, 0.0, THRESHOLD);
  EXPECT_NEAR(cmds.rear_steering_motor_angle, 0.0, THRESHOLD);
}

TEST_F(TestFourWheelSteeringModel, test_correct_distribution_per_direction)
{
  FourWheelSteering cmd_4ws;
  cmd_4ws.speed = 1.0;
  cmd_4ws.front_steering_angle = M_PI_4 / 2;
  cmd_4ws.rear_steering_angle = -M_PI_4 / 2;

  auto cmds = model_->compute_commands(cmd_4ws);
  EXPECT_LT(cmds.front_left_motor_radial_vel, cmds.front_right_motor_radial_vel);
  EXPECT_LT(cmds.rear_left_motor_radial_vel, cmds.rear_right_motor_radial_vel);
  EXPECT_GT(cmds.front_left_steering_angle, cmds.front_right_steering_angle);
}

TEST_F(TestFourWheelSteeringModel, test_state_match_command_left_turn)
{
  constexpr double THRESHOLD = 1e-12;

  FourWheelSteeringState state;
  FourWheelSteering cmd_4ws;
  cmd_4ws.speed = 1.0;
  cmd_4ws.front_steering_angle = M_PI_4 / 2;
  cmd_4ws.rear_steering_angle = -M_PI_4 / 2;

  auto cmds = model_->compute_commands(cmd_4ws);
  memcpy(&state, &cmds, sizeof(double) * 8);
  auto odom = model_->compute_odometry(state);

  EXPECT_NEAR(odom.speed, cmd_4ws.speed, THRESHOLD);
  EXPECT_NEAR(odom.front_steering_angle, cmd_4ws.front_steering_angle, THRESHOLD);
  EXPECT_NEAR(odom.rear_steering_angle, cmd_4ws.rear_steering_angle, THRESHOLD);
}

TEST_F(TestFourWheelSteeringModel, test_state_match_command_right_turn)
{
  constexpr double THRESHOLD = 1e-12;

  FourWheelSteeringState state;
  FourWheelSteering cmd_4ws;
  cmd_4ws.speed = 1.0;
  cmd_4ws.front_steering_angle = -M_PI_4 / 2;
  cmd_4ws.rear_steering_angle = M_PI_4 / 2;

  auto cmds = model_->compute_commands(cmd_4ws);
  memcpy(&state, &cmds, sizeof(double) * 8);
  auto odom = model_->compute_odometry(state);

  EXPECT_NEAR(odom.speed, cmd_4ws.speed, THRESHOLD);
  EXPECT_NEAR(odom.front_steering_angle, cmd_4ws.front_steering_angle, THRESHOLD);
  EXPECT_NEAR(odom.rear_steering_angle, cmd_4ws.rear_steering_angle, THRESHOLD);
}
