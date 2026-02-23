#include <boost/polymorphic_pointer_cast.hpp>

#include <tobas_constants/path.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp>
#include <tobas_fc1xx_core/dshot.hpp>
#include <tobas_node/node.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_tools/control_latency_publisher.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/srv/get_rotor_control_gains.hpp>
#include <tobas_msgs/srv/set_rotor_control_gains.hpp>

using namespace std::chrono_literals;

namespace fs = std::filesystem;

class DShotDriverNode : public tobas::BaseNode
{
  using self = DShotDriverNode;
  using super = tobas::BaseNode;

  using GetGains = tobas_msgs::srv::GetRotorControlGains;
  using SetGains = tobas_msgs::srv::SetRotorControlGains;
  using SaveGains = std_srvs::srv::Trigger;

  static constexpr char kGainKeyPrefix[] = "speed_control_gain_";

public:
  explicit DShotDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  fc1xx::DShot dshot_;

  ptree::PropertyTree pt_;
  std::array<uint8_t, fc1xx::DShot::kChannelSize> gains_ = {};
  bool is_commanded_ = false;
  tobas::ElectricPropulsionSystemConfig::ConstSharedPtr eprop_;

  ros2::PublisherPtr<tobas_msgs::msg::RotorStateArray> rotor_states_pub_;
  tobas::ControlLatencyPublisher latency_pub_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_sub_;

  ros2::ServiceServerPtr<GetGains> get_gains_ss_;
  ros2::ServiceServerPtr<SetGains> set_gains_ss_;
  ros2::ServiceServerPtr<SaveGains> save_gains_ss_;

  ros2::TimerPtr auto_stop_timer_;

  bool transfer();
  bool transferAndSleep();
  void publishCurrentRotorStates();
  void publishErrorRotorStates();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds);

  void getGainsCb(const GetGains::Request::ConstSharedPtr& req, const GetGains::Response::SharedPtr& res);
  void setGainsCb(const SetGains::Request::ConstSharedPtr& req, const SetGains::Response::SharedPtr& res);
  void saveGainsCb(const SaveGains::Request::ConstSharedPtr& req, const SaveGains::Response::SharedPtr& res);

  void autoStopTimerCb();
};

DShotDriverNode::DShotDriverNode(const rclcpp::NodeOptions& options) : super("fc1xx_dshot_driver", options)
{
  if (!pt_.initialize((fs::path(tobas::kConfigDirRoot) / "dshot.json"))) {
    TOBAS_ERROR("Failed to initialize property tree. This node will not work.");
    return;
  }

  drone_sub_ = createSubscriber(tobas::topic::kDrone, &self::droneCb, this, true, true);
}

bool DShotDriverNode::transfer()
{
  if (!dshot_.transfer()) {
    TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Failed to communicate with the rotor controller.");
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
    const auto erotor = boost::polymorphic_pointer_downcast<tobas::ElectricRotorConfig>(rotor);
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
    const auto erotor = boost::polymorphic_pointer_downcast<tobas::ElectricRotorConfig>(rotor);
    rotor_states->states.emplace_back();
    rotor_states->states.back().link_name = rotor->link_name;
    rotor_states->states.back().speed = NAN;
    rotor_states->states.back().thrust = NAN;
    rotor_states->states.back().status = tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE;
  }

  rotor_states_pub_->publish(std::move(rotor_states));
}

void DShotDriverNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  if (eprop_) {
    TOBAS_WARN("DShot driver cannot be re-initialized.");
    return;
  }

  if (!drone->prop) {
    return;
  }
  if (drone->prop->type() != tobas::PropulsionSystem::kElectric) {
    return;
  }

  const auto eprop = boost::polymorphic_pointer_downcast<tobas::ElectricPropulsionSystemConfig>(drone->prop);

  // Initialize DShot driver
  if (!dshot_.initialize()) {
    TOBAS_ERROR("Failed to initialize DShot driver.");
    return;
  }

  // Set Kv values
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

  // Set internal resistances
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

  // Set propeller diameters
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

  // Set moment constants
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

  // Set the number of poles
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

  // Load and set the speed control gains
  for (const auto& [link_name, _] : eprop->rotors) {
    const auto erotor = eprop->getRotor(link_name);
    if (erotor->channel >= fc1xx::DShot::kChannelSize) {
      TOBAS_ERROR("Rotor channel ", erotor->channel, " is out of range.");
      continue;
    }
    if (!pt_.get(ns(), kGainKeyPrefix + std::to_string(erotor->channel), gains_.at(erotor->channel))) {
      TOBAS_ERROR("Failed to load the rotor speed control gain of channel ", erotor->channel, ".");
      continue;
    }
    if (!dshot_.setSpeedControlGain(erotor->channel, gains_.at(erotor->channel))) {
      TOBAS_ERROR("Failed to set the rotor speed control gain of channel ", erotor->channel, ".");
      continue;
    }
  }
  if (!transferAndSleep()) {
    return;
  }

  // Resister publishers
  rotor_states_pub_ = createPublisher<tobas_msgs::msg::RotorStateArray>(tobas::topic::kRotorStates);
  latency_pub_.initialize(shared_from_this());

  // Resister subscribers
  tar_speeds_sub_ = createSubscriber(tobas::topic::kRotorSpeedsCmd, &self::targetSpeedsCb, this);

  // Resister service servers
  get_gains_ss_ = createService<GetGains>(tobas::service::kGetRotorControlGains, &self::getGainsCb, this);
  set_gains_ss_ = createService<SetGains>(tobas::service::kSetRotorControlGains, &self::setGainsCb, this);
  save_gains_ss_ = createService<SaveGains>(tobas::service::kSaveRotorControlGains, &self::saveGainsCb, this);

  // Create timers
  auto_stop_timer_ = createWallTimer(tobas::kCommandAutoResetTimeout, &self::autoStopTimerCb, this);

  eprop_ = eprop;
  TOBAS_INFO("Rotor speed controller is initialized.");
}

void DShotDriverNode::targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds)
{
  // Set target speeds of each channel
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

  // Send the commands and publish the rotor states
  // NOTE: Even in the event of a communication error, the motor status must always be published.
  if (transfer()) {
    publishCurrentRotorStates();
  }
  else {
    publishErrorRotorStates();
  }

  // Publish the control latency
  latency_pub_.publish(tar_speeds->header.stamp);

  // Reset the timeout timer
  auto_stop_timer_->reset();

  // Now the rotors are commanded
  is_commanded_ = true;
}

void DShotDriverNode::getGainsCb(const GetGains::Request::ConstSharedPtr&, const GetGains::Response::SharedPtr& res)
{
  res->gains.assign(gains_.begin(), gains_.end());
  res->success = true;
}

void DShotDriverNode::setGainsCb(const SetGains::Request::ConstSharedPtr& req, const SetGains::Response::SharedPtr& res)
{
  for (const auto& gain : req->gains) {
    if (!dshot_.setSpeedControlGain(gain.channel, gain.gain)) {
      res->success = false;
      res->message = "Rotor control gain of channel " + std::to_string((int)gain.channel) + " is rejected.";
      return;
    }
    gains_.at(gain.channel) = gain.gain;
  }

  if (!transfer()) {
    res->success = false;
    res->message = "Failed to communicate with the rotor controller.";
    return;
  }

  res->success = true;
  res->message.clear();
}

void DShotDriverNode::saveGainsCb(const SaveGains::Request::ConstSharedPtr&, const SaveGains::Response::SharedPtr& res)
{
  for (size_t ch = 0; ch < fc1xx::DShot::kChannelSize; ++ch) {
    const auto key = kGainKeyPrefix + std::to_string(ch);
    pt_.set(ns(), key, gains_.at(ch));
  }

  if (!pt_.save()) {
    res->success = false;
    res->message = "Failed to save gains.";
    return;
  }

  res->success = true;
  res->message.clear();
}

void DShotDriverNode::autoStopTimerCb()
{
  for (size_t ch = 0; ch < fc1xx::DShot::kChannelSize; ++ch) {
    if (!dshot_.setThrottle(ch, fc1xx::DShot::DSHOT_CMD_MOTOR_STOP)) {
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
      tobas::kCommandAutoResetTimeout,
      " have elapsed since the last command.");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(DShotDriverNode)
