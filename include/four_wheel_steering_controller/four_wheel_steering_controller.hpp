// Copyright 2020 PAL Robotics S.L.
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

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_publisher.h"

namespace four_wheel_steering_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class handle_configuration_exception : public std::exception
{
private:
  std::string message_;

public:
  explicit handle_configuration_exception(const std::string & message);
  const char * what() const noexcept override
  {
    return message_.c_str();
  }
};

handle_configuration_exception::handle_configuration_exception(const std::string & message)
: message_(message)
{

}

class FourWheelSteeringController : public controller_interface::ControllerInterface
{
  using FourWheelSteering = four_wheel_steering_msgs::msg::FourWheelSteering;
  using FourWheelSteeringStamped = four_wheel_steering_msgs::msg::FourWheelSteeringStamped;

public:
  FourWheelSteeringController();

  controller_interface::return_type
  init(const std::string & controller_name) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type update() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  struct MotorHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command;
  };
  struct SteeringHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> left_kingpin;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> right_kingpin;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position;
  };
protected:
  std::unique_ptr<MotorHandle> front_left_motor_handle_;
  std::unique_ptr<MotorHandle> front_right_motor_handle_;
  std::unique_ptr<MotorHandle> rear_left_motor_handle_;
  std::unique_ptr<MotorHandle> rear_right_motor_handle_;
  std::unique_ptr<SteeringHandle> front_steering_handle_;
  std::unique_ptr<SteeringHandle> rear_steering_handle_;

  struct VehicleParams
  {
    double wheel_base = 0.0;
    double wheel_track = 0.0;
    double wheel_radius = 0.0;
    double distance_steering_to_wheel = 0.0;
    double steering_gear_transmission_ratio = 0.0;
  } vehicle_params_;

  struct OdometryParams
  {
    std::string frame_id = "base_link";
  } odom_params_;

  rclcpp::Publisher<FourWheelSteeringStamped>::SharedPtr pub_odom_;
  realtime_tools::RealtimePublisherSharedPtr<FourWheelSteeringStamped> pub_odom_realtime_;

  // Timeout after which commands are considered too old to apply
  std::chrono::milliseconds cmd_timeout_{500};

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<FourWheelSteeringStamped>::SharedPtr sub_cmd_;

  realtime_tools::RealtimeBox<std::shared_ptr<FourWheelSteeringStamped>> received_cmd_msg_ptr_{
    nullptr};

  std::queue<FourWheelSteeringStamped> cmd_queue_;
  rclcpp::Time previous_update_timestamp_{0};

  bool is_halted = false;

  std::unique_ptr<MotorHandle> configure_motor(const std::string & name);
  std::unique_ptr<SteeringHandle> configure_steering(const std::string & name);
  void set_command(const FourWheelSteering&) const;
  bool reset();
  void halt();
};

}  // namespace four_wheel_steering_controller
