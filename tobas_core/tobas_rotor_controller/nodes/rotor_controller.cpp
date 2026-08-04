// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <boost/polymorphic_pointer_cast.hpp>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_std_tools/vector.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

using namespace std::chrono_literals;

namespace tobas
{
/* Realize the target thrust for the propulsion system. */
class RotorControllerNode : public BaseNode
{
  using self = RotorControllerNode;
  using super = BaseNode;

  using SetArm = tobas_msgs::srv::SetArm;

  static constexpr auto kPublishArmingPeriod = 1s;
  static constexpr auto kAutoDisarmBeforeCmdTimeout = 10s;
  static constexpr auto kAutoDisarmAfterCmdTimeout = 500ms;

public:
  explicit RotorControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::VehicleHealth::ConstSharedPtr health_;

  bool is_armed_ = false;
  bool is_commanded_ = false;

  ros2::PublisherPtr<tobas_msgs::msg::RotorSpeedArray> rotor_speeds_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::Arming> arming_pub_;

  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::VehicleHealth> health_sub_;

  ros2::ServiceServerPtr<SetArm> set_arm_ss_;

  ros2::TimerPtr publish_arming_timer_;
  ros2::TimerPtr auto_disarm_timer_;

  void publishCurrentArmingState();
  void publishZeroThrottle();
  void disarm();

  void droneCb(const Drone::ConstSharedPtr& drone);
  void thrustsCmdCb(const tobas_msgs::msg::RotorThrustArray::ConstSharedPtr& tar_thrusts_msg);
  void healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health);

  void setArmCb(const SetArm::Request::ConstSharedPtr& req, const SetArm::Response::SharedPtr& res);

  void autoDisarmBeforeCmdTimerCb();
  void autoDisarmAfterCmdTimerCb();
};

RotorControllerNode::RotorControllerNode(const rclcpp::NodeOptions& options)
  : super("rotor_controller", nodeOptions_Default(options))
{
  rotor_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeedArray>(topic::kRotorSpeedsCmd);
  ice_cmd_pub_ = createPublisher<tobas_msgs::msg::IcePropulsionSystemCommand>(topic::kIcePropulsionSystemCmd);
  arming_pub_ = createPublisher<tobas_msgs::msg::Arming>(topic::kArming);

  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  tar_thrusts_sub_ = createSubscriber(topic::kRotorThrustsCmd, &self::thrustsCmdCb, this);
  health_sub_ = createSubscriber(topic::kVehicleHealth, &self::healthCb, this);

  set_arm_ss_ = createService<SetArm>(service::kSetArm, &self::setArmCb, this);

  publish_arming_timer_ = createTimer(kPublishArmingPeriod, &self::publishCurrentArmingState, this);
}

void RotorControllerNode::publishCurrentArmingState()
{
  auto arming_msg = std::make_unique<tobas_msgs::msg::Arming>();
  arming_msg->header.stamp = now();
  arming_msg->data = is_armed_;
  arming_pub_->publish(std::move(arming_msg));
}

void RotorControllerNode::publishZeroThrottle()
{
  switch (drone_->prop->type()) {
    case PropulsionSystem::kElectric: {
      const auto eprop = boost::polymorphic_pointer_downcast<ElectricPropulsionSystemConfig>(drone_->prop);

      auto tar_speeds_msg = std::make_unique<tobas_msgs::msg::RotorSpeedArray>();
      tar_speeds_msg->header.stamp = now();
      for (const auto& [link_name, rotor] : eprop->rotors) {
        const auto erotor = boost::polymorphic_pointer_downcast<ElectricRotorConfig>(rotor);
        tar_speeds_msg->speeds.emplace_back();
        tar_speeds_msg->speeds.back().link_name = link_name;
        tar_speeds_msg->speeds.back().speed = 0.0;
      }

      rotor_speeds_pub_->publish(std::move(tar_speeds_msg));

      break;
    }
    case PropulsionSystem::kIce: {
      const auto iprop = boost::polymorphic_pointer_downcast<IcePropulsionSystemConfig>(drone_->prop);

      auto ice_cmd_msg = std::make_unique<tobas_msgs::msg::IcePropulsionSystemCommand>();
      ice_cmd_msg->header.stamp = now();
      ice_cmd_msg->engine_throttle = 0.0;
      for (const auto& [link_name, rotor] : iprop->rotors) {
        const auto irotor = boost::polymorphic_pointer_downcast<IceRotorConfig>(rotor);
        ice_cmd_msg->pitch_angles.emplace_back();
        ice_cmd_msg->pitch_angles.back().link_name = link_name;
        ice_cmd_msg->pitch_angles.back().angle = irotor->center_pitch;
      }

      ice_cmd_pub_->publish(std::move(ice_cmd_msg));

      break;
    }
    default: {
      TOBAS_ERROR("Invalid propulsion system type: ", (int)drone_->prop->type());
      return;
    }
  }
}

void RotorControllerNode::disarm()
{
  publishZeroThrottle();

  is_armed_ = false;
  is_commanded_ = false;

  publishCurrentArmingState();
  publish_arming_timer_->reset();

  auto_disarm_timer_.reset();
}

void RotorControllerNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
}

void RotorControllerNode::thrustsCmdCb(const tobas_msgs::msg::RotorThrustArray::ConstSharedPtr& tar_thrusts_msg)
{
  if (!drone_) {
    return;
  }

  if (!is_armed_) {
    TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Command is ignored because the vehicle is disarmed.");
    return;
  }

  switch (drone_->prop->type()) {
    case PropulsionSystem::kElectric: {
      const auto eprop = boost::polymorphic_pointer_downcast<ElectricPropulsionSystemConfig>(drone_->prop);

      // Create target speeds message.
      auto tar_speeds_msg = std::make_unique<tobas_msgs::msg::RotorSpeedArray>();
      tar_speeds_msg->header = tar_thrusts_msg->header;

      // Convert target thrusts to target speeds.
      for (const auto& elem : tar_thrusts_msg->thrusts) {
        const auto erotor = eprop->getRotor(elem.link_name);
        if (!erotor) {
          TOBAS_ERROR("Electric rotor \"" + elem.link_name + "\" does not exist.");
          continue;
        }
        const auto tar_thrust = std::max(elem.thrust, 0.0);
        const auto tar_speed = erotor->speedFromThrust(tar_thrust);
        tar_speeds_msg->speeds.emplace_back();
        tar_speeds_msg->speeds.back().link_name = elem.link_name;
        tar_speeds_msg->speeds.back().speed = std::max(tar_speed, erotor->min_speed);
      }

      // Publish target speeds.
      rotor_speeds_pub_->publish(std::move(tar_speeds_msg));

      break;
    }
    case PropulsionSystem::kIce: {  // Realize thrust using the reference pitch angle (memo: 3-27)
      const auto iprop = boost::polymorphic_pointer_downcast<IcePropulsionSystemConfig>(drone_->prop);

      // Calculate the total torque applied to the engine shaft and its coefficient.
      double thrust_sum = 0.0;
      double torque_sum = 0.0;
      double K = 0.0;
      for (const auto& elem : tar_thrusts_msg->thrusts) {
        const auto irotor = iprop->getRotor(elem.link_name);
        if (!irotor) {
          TOBAS_ERROR("ICE Rotor \"" + elem.link_name + "\" does not exist.");
          continue;
        }
        const auto motor_const = irotor->motorConst(irotor->center_pitch);
        const auto moment_const = irotor->momentConst(irotor->center_pitch);
        const auto tar_thrust = std::max(elem.thrust, 0.0);
        thrust_sum += tar_thrust;
        torque_sum += moment_const * tar_thrust / irotor->gear_ratio;  // Account for gear reduction.
        K += motor_const * moment_const / math::cube(irotor->gear_ratio);
      }

      // Create command.
      auto ice_cmd_msg = std::make_unique<tobas_msgs::msg::IcePropulsionSystemCommand>();
      ice_cmd_msg->header = tar_thrusts_msg->header;

      // Determine the engine throttle and propeller pitch angles.
      if (thrust_sum <= 0.0) {
        ice_cmd_msg->engine_throttle = 0.0;
        for (const auto& elem : tar_thrusts_msg->thrusts) {
          const auto irotor = iprop->getRotor(elem.link_name);
          ice_cmd_msg->pitch_angles.emplace_back();
          ice_cmd_msg->pitch_angles.back().link_name = elem.link_name;
          ice_cmd_msg->pitch_angles.back().angle = irotor->center_pitch;
        }
      }
      else {
        const auto engine_speed = std::sqrt(torque_sum / K);
        ice_cmd_msg->engine_throttle = iprop->engine.computeThrottle(torque_sum, engine_speed);
        for (const auto& elem : tar_thrusts_msg->thrusts) {
          const auto irotor = iprop->getRotor(elem.link_name);
          const auto tar_thrust = std::max(elem.thrust, 0.0);
          ice_cmd_msg->pitch_angles.emplace_back();
          ice_cmd_msg->pitch_angles.back().link_name = elem.link_name;
          ice_cmd_msg->pitch_angles.back().angle = irotor->pitchFromThrust(engine_speed, tar_thrust);
        }
      }

      // Publish command.
      ice_cmd_pub_->publish(std::move(ice_cmd_msg));

      break;
    }
    default: {
      TOBAS_ERROR("Invalid propulsion system type: ", (int)drone_->prop->type());
      return;
    }
  }

  // Reset timeout timers.
  if (is_commanded_) {
    auto_disarm_timer_->reset();
  }
  else {
    is_commanded_ = true;
    auto_disarm_timer_ = createTimer(kAutoDisarmAfterCmdTimeout, &self::autoDisarmAfterCmdTimerCb, this);
  }
}

void RotorControllerNode::healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health)
{
  health_ = health;
}

void RotorControllerNode::setArmCb(const SetArm::Request::ConstSharedPtr& req, const SetArm::Response::SharedPtr& res)
{
  if (!is_armed_ && req->arming) {
    if (!health_) {
      res->success = false;
      res->message = "Vehicle health status has not been received yet.";
      return;
    }

    if (!health_->ok) {
      res->success = false;
      res->message = "Pre-arm check failed.";
      return;
    }

    is_armed_ = true;
    publishCurrentArmingState();
    auto_disarm_timer_ = createTimer(kAutoDisarmBeforeCmdTimeout, &self::autoDisarmBeforeCmdTimerCb, this);
    TOBAS_INFO("Arming request was accepted.");
  }
  else if (is_armed_ && !req->arming) {
    disarm();
    TOBAS_INFO("Disarming request was accepted.");
  }

  res->success = true;
  res->message.clear();
}

void RotorControllerNode::autoDisarmBeforeCmdTimerCb()
{
  disarm();

  TOBAS_WARN(
    "All rotors have been automatically disarmed because ",
    kAutoDisarmBeforeCmdTimeout,
    " have elapsed since arming with no commands received.");
}

void RotorControllerNode::autoDisarmAfterCmdTimerCb()
{
  disarm();

  TOBAS_WARN(
    "All rotors have been automatically disarmed because ",
    kAutoDisarmAfterCmdTimeout,
    " have elapsed since the last command.");
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::RotorControllerNode)
