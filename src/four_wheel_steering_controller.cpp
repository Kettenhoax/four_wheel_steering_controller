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

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <limits>

#include "four_wheel_steering_controller/four_wheel_steering_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "cmd_4ws";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "odom_4ws";
}

namespace four_wheel_steering_controller
{
using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using four_wheel_steering_msgs::msg::FourWheelSteering;
using four_wheel_steering_msgs::msg::FourWheelSteeringStamped;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

FourWheelSteeringController::FourWheelSteeringController()
: controller_interface::ControllerInterface(), cmd_timeout_(500ms) {}

controller_interface::return_type
FourWheelSteeringController::init(const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  try {
    auto node = get_node();
    node->declare_parameter<double>("wheel_base", vehicle_params_.wheel_base);
    node->declare_parameter<double>("wheel_track", vehicle_params_.wheel_track);
    node->declare_parameter<double>("wheel_radius", vehicle_params_.wheel_radius);
    node->declare_parameter<double>(
      "distance_steering_to_wheel",
      vehicle_params_.distance_steering_to_wheel);
    node->declare_parameter<double>(
      "steering_gear_transmission_ratio",
      vehicle_params_.steering_gear_transmission_ratio);

    node->declare_parameter<std::string>("odom_frame_id", odom_params_.frame_id);
    node->declare_parameter<int>("odom_frequency_offset", 1);
    node->declare_parameter<double>("cmd_timeout", static_cast<double>(cmd_timeout_.seconds()));
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

InterfaceConfiguration FourWheelSteeringController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (auto joint_name : {
      "front_left_motor",
      "front_right_motor",
      "rear_left_motor",
      "rear_right_motor",
    })
  {
    std::stringstream ss;
    ss << joint_name << "/" << HW_IF_VELOCITY;
    conf_names.push_back(ss.str());
  }
  for (auto joint_name : {
      "front_steering",
      "rear_steering",
    })
  {
    std::stringstream ss;
    ss << joint_name << "/" << HW_IF_POSITION;
    conf_names.push_back(ss.str());
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration FourWheelSteeringController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  const auto & motor_names = {
    "front_left_motor",
    "front_right_motor",
    "rear_left_motor",
    "rear_right_motor",
  };
  const auto & steering_names = {
    "front_left_kingpin",
    "front_right_kingpin",
    "rear_left_kingpin",
    "rear_right_kingpin",
  };
  for (const auto & joint_name : motor_names) {
    std::stringstream ss;
    ss << joint_name << "/" << HW_IF_VELOCITY;
    conf_names.push_back(ss.str());
  }
  for (const auto & joint_name : steering_names) {
    std::stringstream ss;
    ss << joint_name << "/" << HW_IF_POSITION;
    conf_names.push_back(ss.str());
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type FourWheelSteeringController::update()
{
  auto logger = node_->get_logger();
  auto clock = node_->get_clock();
  if (get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  const auto current_time = clock->now();

  FourWheelSteeringStamped::SharedPtr last_msg;
  received_cmd_msg_ptr_.get(last_msg);

  if (last_msg == nullptr) {
    RCLCPP_WARN(logger, "Command message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto dt = current_time - last_msg->header.stamp;
  if (dt > cmd_timeout_) {
    halt();
  } else if (dt.nanoseconds() < -cmd_timeout_.nanoseconds()) {
    RCLCPP_WARN_THROTTLE(
      logger, *clock, (1000ms).count(), "command age is negative, are %s timestamps in ROS time?", 
      sub_cmd_->get_topic_name());
    halt();
  } else {
    auto cmds_4ws = vehicle_model_->compute_commands(last_msg->data);
    front_steering_handle_->position.get().set_value(cmds_4ws.front_steering_motor_angle);
    rear_steering_handle_->position.get().set_value(cmds_4ws.rear_steering_motor_angle);
    front_left_motor_handle_->command.get().set_value(cmds_4ws.front_left_motor_radial_vel);
    front_right_motor_handle_->command.get().set_value(cmds_4ws.front_right_motor_radial_vel);
    rear_left_motor_handle_->command.get().set_value(cmds_4ws.rear_left_motor_radial_vel);
    rear_right_motor_handle_->command.get().set_value(cmds_4ws.rear_right_motor_radial_vel);
  }

  FourWheelSteeringState state_4ws;
  state_4ws.front_left_steering_angle = front_steering_handle_->left_kingpin.get().get_value();
  state_4ws.front_right_steering_angle = front_steering_handle_->right_kingpin.get().get_value();
  state_4ws.rear_left_steering_angle = rear_steering_handle_->left_kingpin.get().get_value();
  state_4ws.rear_right_steering_angle = rear_steering_handle_->right_kingpin.get().get_value();
  state_4ws.front_left_motor_radial_vel = front_left_motor_handle_->state.get().get_value();
  state_4ws.front_right_motor_radial_vel = front_right_motor_handle_->state.get().get_value();
  state_4ws.rear_left_motor_radial_vel = rear_left_motor_handle_->state.get().get_value();
  state_4ws.rear_right_motor_radial_vel = rear_right_motor_handle_->state.get().get_value();
  auto odom_4ws = vehicle_model_->compute_odometry(state_4ws);

  if (0 == (update_index_ % odom_params_.frequency_offset)) {
    if (pub_odom_realtime_->trylock()) {
      auto & odometry_message = pub_odom_realtime_->msg_;
      odometry_message.header.stamp = current_time;
      odometry_message.data = odom_4ws;
      pub_odom_realtime_->unlockAndPublish();
    }
  }
  update_index_++;

  return controller_interface::return_type::OK;
}

CallbackReturn FourWheelSteeringController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = node_->get_logger();

  vehicle_params_.wheel_base = node_->get_parameter("wheel_base").as_double();
  vehicle_params_.wheel_track = node_->get_parameter("wheel_track").as_double();
  vehicle_params_.wheel_radius = node_->get_parameter("wheel_radius").as_double();
  vehicle_params_.distance_steering_to_wheel =
    node_->get_parameter("distance_steering_to_wheel").as_double();
  vehicle_params_.steering_gear_transmission_ratio = node_->get_parameter(
    "steering_gear_transmission_ratio").as_double();
  try {
    vehicle_model_ = std::make_unique<GenericFourWheelSteering>(vehicle_params_);
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(logger, e.what());
    return CallbackReturn::ERROR;
  }

  odom_params_.frame_id = node_->get_parameter("odom_frame_id").as_string();
  odom_params_.frequency_offset = node_->get_parameter("odom_frequency_offset").as_int();
  if (odom_params_.frequency_offset < 1) {
    RCLCPP_ERROR(logger, "odom_frequency_offset must be a positive integer");
    return CallbackReturn::ERROR;
  }
  cmd_timeout_ = rclcpp::Duration::from_seconds(node_->get_parameter("cmd_timeout").as_double());
  if (!reset()) {
    return CallbackReturn::ERROR;
  }

  const FourWheelSteeringStamped empty_cmd;
  received_cmd_msg_ptr_.set(std::make_shared<FourWheelSteeringStamped>(empty_cmd));

  sub_cmd_ = node_->create_subscription<FourWheelSteeringStamped>(
    DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), [this](
      const std::shared_ptr<FourWheelSteeringStamped> msg) -> void {
      if (!subscriber_is_active_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Can't accept new commands, subscriber is inactive");
        return;
      }
      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
        RCLCPP_WARN_ONCE(
          node_->get_logger(),
          "Received FourWheelSteeringStamped with zero timestamp, setting it to current "
          "time, this message will only be shown once");
        msg->header.stamp = node_->get_clock()->now();
      }
      received_cmd_msg_ptr_.set(std::move(msg));
    });

  pub_odom_ =
    node_->create_publisher<FourWheelSteeringStamped>(
    DEFAULT_ODOMETRY_TOPIC,
    rclcpp::SystemDefaultsQoS());
  pub_odom_realtime_ =
    std::make_shared<
    realtime_tools::RealtimePublisher<FourWheelSteeringStamped>>(pub_odom_);

  auto & odometry_message = pub_odom_realtime_->msg_;
  odometry_message.header.frame_id = odom_params_.frame_id;

  // initialize odom values zeros
  odometry_message.data = FourWheelSteering(rosidl_runtime_cpp::MessageInitialization::ALL);
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelSteeringController::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    front_left_motor_handle_ = configure_motor("front_left_motor");
    front_right_motor_handle_ = configure_motor("front_right_motor");
    rear_left_motor_handle_ = configure_motor("rear_left_motor");
    rear_right_motor_handle_ = configure_motor("rear_right_motor");
    front_steering_handle_ = configure_steering("front");
    rear_steering_handle_ = configure_steering("rear");
  } catch (const handle_configuration_exception & e) {
    RCLCPP_ERROR(node_->get_logger(), e.what());
    return CallbackReturn::ERROR;
  }
  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelSteeringController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelSteeringController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return CallbackReturn::ERROR;
  }

  received_cmd_msg_ptr_.set(std::make_shared<FourWheelSteeringStamped>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelSteeringController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool FourWheelSteeringController::reset()
{
  front_left_motor_handle_.reset();
  front_right_motor_handle_.reset();
  rear_left_motor_handle_.reset();
  rear_right_motor_handle_.reset();
  front_steering_handle_.reset();
  rear_steering_handle_.reset();

  subscriber_is_active_ = false;
  sub_cmd_.reset();

  received_cmd_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

CallbackReturn FourWheelSteeringController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void FourWheelSteeringController::halt()
{
  front_left_motor_handle_->command.get().set_value(0.0);
  front_right_motor_handle_->command.get().set_value(0.0);
  rear_left_motor_handle_->command.get().set_value(0.0);
  rear_right_motor_handle_->command.get().set_value(0.0);
  front_steering_handle_->position.get().set_value(0.0);
  rear_steering_handle_->position.get().set_value(0.0);
}

std::unique_ptr<FourWheelSteeringController::MotorHandle> FourWheelSteeringController::
configure_motor(const std::string & name)
{
  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [&name](const auto & interface) {
      return interface.get_name() == name &&
      interface.get_interface_name() == HW_IF_VELOCITY;
    });

  if (state_handle == state_interfaces_.cend()) {
    throw handle_configuration_exception("Unable to obtain joint state handle for " + name);
  }

  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(), [&name](
      const auto & interface) {
      return interface.get_name() == name &&
      interface.get_interface_name() == HW_IF_VELOCITY;
    });

  if (command_handle == command_interfaces_.end()) {
    throw handle_configuration_exception("Unable to obtain joint command handle for " + name);
  }

  return std::unique_ptr<MotorHandle> {new MotorHandle{std::ref(*state_handle), std::ref(
        *command_handle)}};
}

std::unique_ptr<FourWheelSteeringController::SteeringHandle> FourWheelSteeringController::
configure_steering(
  const std::string & name)
{
  auto left_kingpin_name = name + "_left_kingpin";
  const auto left_kingpin_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&left_kingpin_name](const auto & interface) {
      return interface.get_name() == left_kingpin_name &&
      interface.get_interface_name() == HW_IF_POSITION;
    });

  if (left_kingpin_handle == state_interfaces_.cend()) {
    throw handle_configuration_exception(
            "Unable to obtain state handle for " + left_kingpin_name);
  }

  auto right_kingpin_name = name + "_right_kingpin";
  const auto right_kingpin_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&right_kingpin_name](const auto & interface) {
      return interface.get_name() == right_kingpin_name &&
      interface.get_interface_name() == HW_IF_POSITION;
    });

  if (right_kingpin_handle == state_interfaces_.cend()) {
    throw handle_configuration_exception(
            "Unable to obtain state handle for " + right_kingpin_name);
  }

  auto steering_name = name + "_steering";
  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(), [&steering_name](
      const auto & interface) {
      return interface.get_name() == steering_name &&
      interface.get_interface_name() == HW_IF_POSITION;
    });

  if (command_handle == command_interfaces_.end()) {
    throw handle_configuration_exception(
            "Unable to obtain command handle for " + steering_name);
  }

  return std::unique_ptr<SteeringHandle> {new SteeringHandle{std::ref(*left_kingpin_handle),
      std::ref(*right_kingpin_handle), std::ref(
        *command_handle)}};
}
}  // namespace four_wheel_steering_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  four_wheel_steering_controller::FourWheelSteeringController,
  controller_interface::ControllerInterface)
