// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <boost/polymorphic_pointer_cast.hpp>

#include <tobas_constants/path.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp>
#include <tobas_fc1xx_core/dshot.hpp>
#include <tobas_node/node.hpp>
#include <tobas_tools/control_latency_publisher.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/srv/set_rpm_control_gains.hpp>

using namespace std::chrono_literals;

namespace tobas
{
namespace fc1xx
{
class DShotDriverNode : public BaseNode
{
  using self = DShotDriverNode;
  using super = BaseNode;

  using SetGains = tobas_msgs::srv::SetRpmControlGains;

public:
  explicit DShotDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  DShot dshot_;

  std::array<uint8_t, DShot::kChannelSize> gains_ = {};
  bool is_commanded_ = false;
  ElectricPropulsionSystemConfig::ConstSharedPtr eprop_;

  ros2::PublisherPtr<tobas_msgs::msg::RotorStateArray> rotor_states_pub_;
  ControlLatencyPublisher latency_pub_;

  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_sub_;

  ros2::ServiceServerPtr<SetGains> set_gains_ss_;

  ros2::TimerPtr auto_stop_timer_;

  bool transfer();
  bool transferAndSleep();
  void publishCurrentRotorStates();
  void publishErrorRotorStates();

  void droneCb(const Drone::ConstSharedPtr& drone);
  void targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds);

  void setGainsCb(const SetGains::Request::ConstSharedPtr& req, const SetGains::Response::SharedPtr& res);

  void autoStopTimerCb();
};

DShotDriverNode::DShotDriverNode(const rclcpp::NodeOptions& options)
  : super("fc1xx_dshot_driver", nodeOptions_Default(options)), latency_pub_(this)
{
  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  set_gains_ss_ = createService<SetGains>(service::kSetRpmControlGains, &self::setGainsCb, this);
}

bool DShotDriverNode::transfer()
{
  if (!dshot_.transfer()) {
    TOBAS_ERROR_THROTTLE(kTypicalErrorPeriod, "Failed to communicate with the MCU.");
    return false;
  }
  return true;
}

bool DShotDriverNode::transferAndSleep()
{
  const auto res = transfer();
  rclcpp::sleep_for(1ms);
  return res;
}

void DShotDriverNode::publishCurrentRotorStates()
{
  auto rotor_states = std::make_unique<tobas_msgs::msg::RotorStateArray>();
  rotor_states->header.stamp = now();

  for (const auto& [_, rotor] : eprop_->rotors) {
    const auto erotor = boost::polymorphic_pointer_downcast<ElectricRotorConfig>(rotor);
    rotor_states->states.emplace_back();
    rotor_states->states.back().link_name = rotor->link_name;
    if (dshot_.getValidity(erotor->channel)) {
      const auto speed = dshot_.getSpeed(erotor->channel);
      rotor_states->states.back().speed = speed;
      rotor_states->states.back().thrust = erotor->thrustFromSpeed(speed);
      rotor_states->states.back().status = tobas_msgs::msg::RotorState::NO_ERROR;
    }
    else {
      rotor_states->states.back().speed = NAN;
      rotor_states->states.back().thrust = NAN;
      rotor_states->states.back().status = tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE;
    }
  }

  rotor_states_pub_->publish(std::move(rotor_states));
}

void DShotDriverNode::publishErrorRotorStates()
{
  auto rotor_states = std::make_unique<tobas_msgs::msg::RotorStateArray>();
  rotor_states->header.stamp = now();

  for (const auto& [_, rotor] : eprop_->rotors) {
    const auto erotor = boost::polymorphic_pointer_downcast<ElectricRotorConfig>(rotor);
    rotor_states->states.emplace_back();
    rotor_states->states.back().link_name = rotor->link_name;
    rotor_states->states.back().speed = NAN;
    rotor_states->states.back().thrust = NAN;
    rotor_states->states.back().status = tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE;
  }

  rotor_states_pub_->publish(std::move(rotor_states));
}

void DShotDriverNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  if (eprop_) {
    TOBAS_WARN("DShot driver cannot be re-initialized.");
    return;
  }

  if (!drone->prop) {
    return;
  }
  if (drone->prop->type() != PropulsionSystem::kElectric) {
    return;
  }

  const auto eprop = boost::polymorphic_pointer_downcast<ElectricPropulsionSystemConfig>(drone->prop);

  // Initialize DShot driver.
  if (!dshot_.initialize()) {
    TOBAS_ERROR("Failed to initialize DShot driver.");
    return;
  }

  // Set Kv values.
  for (const auto& [link_name, _] : eprop->rotors) {
    const auto erotor = eprop->getRotor(link_name);
    if (!dshot_.setKv(erotor->channel, erotor->kv)) {
      TOBAS_ERROR("Failed to set Kv of channel ", erotor->channel, ".");
      return;
    }
  }
  if (!transferAndSleep()) {
    return;
  }

  // Set internal resistances.
  for (const auto& [link_name, _] : eprop->rotors) {
    const auto erotor = eprop->getRotor(link_name);
    if (!dshot_.setInternalResistance(erotor->channel, erotor->internal_resistance)) {
      TOBAS_ERROR("Failed to set internal resistance of channel ", erotor->channel, ".");
      return;
    }
  }
  if (!transferAndSleep()) {
    return;
  }

  // Set propeller diameters.
  for (const auto& [link_name, _] : eprop->rotors) {
    const auto erotor = eprop->getRotor(link_name);
    if (!dshot_.setPropellerDiameter(erotor->channel, erotor->propeller_diameter)) {
      TOBAS_ERROR("Failed to set propeller diameter of channel ", erotor->channel, ".");
      return;
    }
  }
  if (!transferAndSleep()) {
    return;
  }

  // Set moment constants.
  for (const auto& [link_name, _] : eprop->rotors) {
    const auto erotor = eprop->getRotor(link_name);
    const auto moment_const = erotor->motor_const * erotor->moment_const / std::pow(erotor->propeller_diameter, 5);
    if (!dshot_.setMomentConstant(erotor->channel, moment_const)) {
      TOBAS_ERROR("Failed to set moment constant of channel ", erotor->channel, ".");
      return;
    }
  }
  if (!transferAndSleep()) {
    return;
  }

  // Set the number of poles.
  for (const auto& [link_name, _] : eprop->rotors) {
    const auto erotor = eprop->getRotor(link_name);
    if (!dshot_.setNumPoles(erotor->channel, erotor->num_poles)) {
      TOBAS_ERROR("Failed to set the number of poles of channel ", erotor->channel, ".");
      return;
    }
  }
  if (!transferAndSleep()) {
    return;
  }

  // Resister publishers.
  rotor_states_pub_ = createPublisher<tobas_msgs::msg::RotorStateArray>(topic::kRotorStates);

  // Resister subscribers.
  tar_speeds_sub_ = createSubscriber(topic::kRotorSpeedsCmd, &self::targetSpeedsCb, this);

  // Create timers.
  auto_stop_timer_ = createWallTimer(kCommandAutoResetTimeout, &self::autoStopTimerCb, this);

  eprop_ = eprop;
  TOBAS_INFO("Rotor speed controller has been initialized.");
}

void DShotDriverNode::targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds)
{
  // Set target speeds of each channel.
  for (const auto& elem : tar_speeds->speeds) {
    const auto erotor = eprop_->getRotor(elem.link_name);
    if (!erotor) {
      TOBAS_ERROR("Rotor \"" + elem.link_name + "\" does not exist.");
      continue;
    }

    if (!dshot_.setTargetSpeed(erotor->channel, elem.speed)) {
      TOBAS_ERROR("Failed to set the target speed of rotor \"", elem.link_name, "\".");
      continue;
    }
  }

  // Send the commands and publish the rotor states.
  // NOTE: Even in the event of a communication error, the motor status must always be published.
  if (transfer()) {
    publishCurrentRotorStates();
  }
  else {
    publishErrorRotorStates();
  }

  // Publish the control latency.
  latency_pub_.publish(tar_speeds->header.stamp);

  // Reset the timeout timer.
  auto_stop_timer_->reset();

  // Now the rotors are commanded.
  is_commanded_ = true;
}

void DShotDriverNode::setGainsCb(const SetGains::Request::ConstSharedPtr& req, const SetGains::Response::SharedPtr& res)
{
  for (const auto& gain : req->gains) {
    if (!dshot_.setRpmControlGain(gain.channel, gain.gain)) {
      res->success = false;
      res->message = "Rotor control gain of channel " + std::to_string((int)gain.channel) + " was rejected.";
      return;
    }
    gains_.at(gain.channel) = gain.gain;
  }

  if (!transfer()) {
    res->success = false;
    res->message = "Failed to communicate with the MCU.";
    return;
  }

  res->success = true;
  res->message.clear();
}

void DShotDriverNode::autoStopTimerCb()
{
  for (size_t ch = 0; ch < DShot::kChannelSize; ++ch) {
    if (!dshot_.setThrottle(ch, DShot::DSHOT_CMD_MOTOR_STOP)) {
      TOBAS_ERROR("Failed to set disarm throttle on channel ", ch, ".");
      return;
    }
  }

  if (transfer()) {
    publishCurrentRotorStates();
  }
  else {
    publishErrorRotorStates();
  }

  if (is_commanded_) {
    is_commanded_ = false;
    TOBAS_INFO(
      "All rotors are automatically stopped because ",
      kCommandAutoResetTimeout,
      " have elapsed since the last command.");
  }
}
}  // namespace fc1xx
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fc1xx::DShotDriverNode)
