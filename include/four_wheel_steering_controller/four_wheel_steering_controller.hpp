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

#include "four_wheel_steering_controller/model.hpp"

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

  struct OdometryParams
  {
    std::string frame_id = "base_link";
    size_t frequency_offset = 1;
  } odom_params_;

  VehicleParams vehicle_params_;
  std::unique_ptr<SteeringModel> vehicle_model_;

  rclcpp::Publisher<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>::SharedPtr pub_odom_;
  realtime_tools::RealtimePublisherSharedPtr<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>
  pub_odom_realtime_;

  // Timeout after which commands are considered too old to apply
  rclcpp::Duration cmd_timeout_;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>::SharedPtr sub_cmd_;

  realtime_tools::RealtimeBox<std::shared_ptr<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>>
  received_cmd_msg_ptr_{
    nullptr};

  size_t update_index_{0};
  bool is_halted = false;

  std::unique_ptr<MotorHandle> configure_motor(const std::string & name);
  std::unique_ptr<SteeringHandle> configure_steering(const std::string & name);
  bool reset();
  void halt();
};

}  // namespace four_wheel_steering_controller
