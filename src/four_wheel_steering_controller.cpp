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
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

static double center_angle(double left, double right)
{
  return atan(2 * tan(left) * tan(right) / (tan(left) + tan(right)));
}

FourWheelSteeringController::FourWheelSteeringController()
: controller_interface::ControllerInterface() {}

controller_interface::return_type
FourWheelSteeringController::init(const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::SUCCESS) {
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

    node->declare_parameter<double>("cmd_timeout", cmd_timeout_.count() / 1000.0);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::SUCCESS;
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

void FourWheelSteeringController::set_command(const FourWheelSteering & cmd_4ws) const
{
  const double tan_front_steering = tan(cmd_4ws.front_steering_angle);
  const double tan_rear_steering = tan(cmd_4ws.rear_steering_angle);

  const double steering_track = vehicle_params_.wheel_track - 2 *
    vehicle_params_.distance_steering_to_wheel;
  const double steering_diff = steering_track * (tan_front_steering - tan_rear_steering) / 2.0;

  auto wb = vehicle_params_.wheel_base;

  double front_left_angle = 0.0;
  double front_right_angle = 0.0;
  double rear_left_angle = 0.0;
  double rear_right_angle = 0.0;

  double front_steering_command = 0.0;
  double rear_steering_command = 0.0;

  if (fabs(vehicle_params_.wheel_base - fabs(steering_diff)) > 0.001) {
    // compute optimal angles resulting from ackermann steering
    front_left_angle = atan(wb * tan_front_steering / (wb - steering_diff));
    front_right_angle = atan(wb * tan_front_steering / (wb + steering_diff));
    rear_left_angle = atan(wb * tan_rear_steering / (wb - steering_diff));
    rear_right_angle = atan(wb * tan_rear_steering / (wb + steering_diff));

    // convert from wheel angle to steering gear angle
    front_steering_command = -vehicle_params_.steering_gear_transmission_ratio *
      cmd_4ws.front_steering_angle;
    rear_steering_command = -vehicle_params_.steering_gear_transmission_ratio *
      cmd_4ws.rear_steering_angle;
  }

  double front_left_motor_command = 0.0;
  double front_right_motor_command = 0.0;
  double rear_left_motor_command = 0.0;
  double rear_right_motor_command = 0.0;

  // determine optimal motor speeds based on steering command
  if (fabs(cmd_4ws.speed) > 0.001) {
    // distance between the projection of the CIR on the wheelbase and the front axle
    double l_front = 0;
    if (fabs(tan(front_left_angle) - tan(front_right_angle)) >
      0.01)
    {
      l_front = tan(front_right_angle) *
        tan(front_left_angle) * steering_track /
        (tan(front_left_angle) - tan(front_right_angle));
    }
    // distance between the projection of the CIR on the wheelbase and the rear axle
    double l_rear = 0;
    if (fabs(tan(rear_left_angle) - tan(rear_right_angle)) > 0.01) {
      l_rear = tan(rear_right_angle) * tan(rear_left_angle) * steering_track /
        (tan(rear_left_angle) - tan(rear_right_angle));
    }

    const double angular_speed_cmd = cmd_4ws.speed * (tan_front_steering - tan_rear_steering) /
      wb;
    const double vel_steering_offset =
      (angular_speed_cmd * vehicle_params_.distance_steering_to_wheel) /
      vehicle_params_.wheel_radius;
    const double sign = copysign(1.0, cmd_4ws.speed);

    front_left_motor_command = sign * std::hypot(
      (cmd_4ws.speed - angular_speed_cmd * steering_track / 2),
      (l_front * angular_speed_cmd)) / vehicle_params_.wheel_radius -
      vel_steering_offset;
    front_right_motor_command = sign * std::hypot(
      (cmd_4ws.speed + angular_speed_cmd * steering_track / 2),
      (l_front * angular_speed_cmd)) / vehicle_params_.wheel_radius +
      vel_steering_offset;
    rear_left_motor_command = sign * std::hypot(
      (cmd_4ws.speed - angular_speed_cmd * steering_track / 2),
      (l_rear * angular_speed_cmd)) / vehicle_params_.wheel_radius -
      vel_steering_offset;
    rear_right_motor_command = sign * std::hypot(
      (cmd_4ws.speed + angular_speed_cmd * steering_track / 2),
      (l_rear * angular_speed_cmd)) / vehicle_params_.wheel_radius +
      vel_steering_offset;
  }

  front_steering_handle_->position.get().set_value(front_steering_command);
  rear_steering_handle_->position.get().set_value(rear_steering_command);

  front_left_motor_handle_->command.get().set_value(front_left_motor_command);
  front_right_motor_handle_->command.get().set_value(front_right_motor_command);
  rear_left_motor_handle_->command.get().set_value(rear_left_motor_command);
  rear_right_motor_handle_->command.get().set_value(rear_right_motor_command);
}

controller_interface::return_type FourWheelSteeringController::update()
{
  auto logger = node_->get_logger();
  if (get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::SUCCESS;
  }

  const auto current_time = node_->get_clock()->now();

  FourWheelSteeringStamped::SharedPtr last_msg;
  received_cmd_msg_ptr_.get(last_msg);

  if (last_msg == nullptr) {
    RCLCPP_WARN(logger, "Command message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto dt = current_time - last_msg->header.stamp;
  // Brake if command has timeout, override the stored command
  if (dt > cmd_timeout_) {
    last_msg->data.speed = 0.0;
    last_msg->data.front_steering_angle = 0.0;
    last_msg->data.rear_steering_angle = 0.0;
  }

  const auto vehicle = vehicle_params_;

  auto front_left_kingpin_angle = front_steering_handle_->left_kingpin.get().get_value();
  auto front_right_kingpin_angle = front_steering_handle_->right_kingpin.get().get_value();
  auto rear_left_kingpin_angle = rear_steering_handle_->left_kingpin.get().get_value();
  auto rear_right_kingpin_angle = rear_steering_handle_->right_kingpin.get().get_value();
  double front_angle = center_angle(front_left_kingpin_angle, front_right_kingpin_angle);
  double rear_angle = center_angle(rear_left_kingpin_angle, rear_right_kingpin_angle);

  auto avg_radps = 0.0;
  avg_radps += front_left_motor_handle_->state.get().get_value();
  avg_radps += front_right_motor_handle_->state.get().get_value();
  avg_radps += rear_left_motor_handle_->state.get().get_value();
  avg_radps += rear_right_motor_handle_->state.get().get_value();
  auto speed = vehicle.wheel_radius * avg_radps / 4;

  if (pub_odom_realtime_->trylock()) {
    auto & odometry_message = pub_odom_realtime_->msg_;
    odometry_message.header.stamp = current_time;
    odometry_message.data.speed = speed;
    odometry_message.data.front_steering_angle = front_angle;
    odometry_message.data.rear_steering_angle = rear_angle;
    pub_odom_realtime_->unlockAndPublish();
  }

  const auto update_dt = current_time - previous_update_timestamp_;
  previous_update_timestamp_ = current_time;

  set_command(last_msg->data);
  return controller_interface::return_type::SUCCESS;
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

  odom_params_.frame_id = node_->get_parameter("odom_frame_id").as_string();
  cmd_timeout_ =
    std::chrono::milliseconds{static_cast<int>(node_->get_parameter("cmd_timeout").as_double() *
    1000.0)};

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

  previous_update_timestamp_ = node_->get_clock()->now();
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
