#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/core.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp>
#include <tobas_tools/control_latency_publisher.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/srv/get_rotor_control_gains.hpp>
#include <tobas_msgs/srv/set_rotor_control_gains.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

#include <tobas_aso_core/dshot.hpp>

using namespace std;
namespace fs = filesystem;

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
  aso::DShot dshot_;

  ptree::PropertyTree pt_;
  array<uint8_t, aso::DShot::kChannelSize> gains_ = { 0 };
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

  bool transferAndSleep();
  void publishRotorStates();
  bool stopRotors();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds);

  void getGainsCb(const GetGains::Request::ConstSharedPtr& req, const GetGains::Response::SharedPtr& res);
  void setGainsCb(const SetGains::Request::ConstSharedPtr& req, const SetGains::Response::SharedPtr& res);
  void saveGainsCb(const SaveGains::Request::ConstSharedPtr& req, const SaveGains::Response::SharedPtr& res);

  void autoStopTimerCb();
};

DShotDriverNode::DShotDriverNode(const rclcpp::NodeOptions& options) : super("aso_dshot_driver", options)
{
  if (!pt_.initialize((fs::path(tobas::kConfigDirRoot) / "dshot.ini")))
  {
    TOBAS_ERROR("Failed to initialize property tree. This node will not work.");
    return;
  }

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
}

bool DShotDriverNode::transferAndSleep()
{
  if (!dshot_.transfer())
  {
    TOBAS_ERROR("SPI communication failed.");
    return false;
  }

  rclcpp::sleep_for(1ms);
  return true;
}

void DShotDriverNode::publishRotorStates()
{
  auto rotor_states = std::make_unique<tobas_msgs::msg::RotorStateArray>();
  rotor_states->header.stamp = get_clock()->now();

  for (const auto& [link_name, _] : eprop_->rotors)
  {
    const auto erotor = eprop_->getRotor(link_name);
    rotor_states->states.emplace_back();
    rotor_states->states.back().link_name = link_name;
    if (dshot_.getValidity(erotor->channel))
    {
      const auto speed = dshot_.getSpeed(erotor->channel);
      rotor_states->states.back().speed = speed;
      rotor_states->states.back().thrust = erotor->thrustFromSpeed(speed);
      rotor_states->states.back().status = tobas_msgs::msg::RotorState::NO_ERROR;
    }
    else
    {
      rotor_states->states.back().speed = NAN;
      rotor_states->states.back().thrust = NAN;
      rotor_states->states.back().status = tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE;
    }
  }

  rotor_states_pub_->publish(move(rotor_states));
}

bool DShotDriverNode::stopRotors()
{
  for (size_t ch = 0; ch < aso::DShot::kChannelSize; ++ch)
  {
    if (!dshot_.setThrottle(ch, aso::DShot::DSHOT_CMD_MOTOR_STOP))
    {
      TOBAS_ERROR("Failed to set disarm throttle on channel ", ch, ".");
      return false;
    }
  }

  if (!dshot_.transfer())
  {
    TOBAS_ERROR("Failed to stop rotors.");
    return false;
  }

  return true;
}

void DShotDriverNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  if (eprop_)
  {
    TOBAS_WARN("DShot driver cannot be re-initialized.");
    return;
  }

  if (!drone->prop)
    return;
  if (drone->prop->type() != tobas::propulsion_system_t::ELECTRIC)
    return;

  const auto eprop = boost::polymorphic_pointer_downcast<tobas::ElectricPropulsionSystemConfig>(drone->prop);

  // Initialize DShot driver
  if (!dshot_.initialize())
  {
    TOBAS_ERROR("Failed to initialize DShot driver.");
    return;
  }

  // Set Kv values
  for (const auto& [link_name, _] : eprop->rotors)
  {
    const auto erotor = eprop->getRotor(link_name);
    if (!dshot_.setKv(erotor->channel, erotor->kv))
    {
      TOBAS_ERROR("Failed to set Kv of channel ", erotor->channel, ".");
      return;
    }
  }
  if (!transferAndSleep())
    return;

  // Set internal resistances
  for (const auto& [link_name, _] : eprop->rotors)
  {
    const auto erotor = eprop->getRotor(link_name);
    if (!dshot_.setInternalResistance(erotor->channel, erotor->internal_resistance))
    {
      TOBAS_ERROR("Failed to set internal resistance of channel ", erotor->channel, ".");
      return;
    }
  }
  if (!transferAndSleep())
    return;

  // Set propeller diameters
  for (const auto& [link_name, _] : eprop->rotors)
  {
    const auto erotor = eprop->getRotor(link_name);
    if (!dshot_.setPropellerDiameter(erotor->channel, erotor->propeller_diameter))
    {
      TOBAS_ERROR("Failed to set propeller diameter of channel ", erotor->channel, ".");
      return;
    }
  }
  if (!transferAndSleep())
    return;

  // Set moment constants
  for (const auto& [link_name, _] : eprop->rotors)
  {
    const auto erotor = eprop->getRotor(link_name);
    const auto moment_const = erotor->motor_const * erotor->moment_const / math::quat(erotor->propeller_diameter);
    if (!dshot_.setMomentConstant(erotor->channel, moment_const))
    {
      TOBAS_ERROR("Failed to set moment constant of channel ", erotor->channel, ".");
      return;
    }
  }
  if (!transferAndSleep())
    return;

  // Set the number of poles
  for (const auto& [link_name, _] : eprop->rotors)
  {
    const auto erotor = eprop->getRotor(link_name);
    if (!dshot_.setNumPoles(erotor->channel, erotor->num_poles))
    {
      TOBAS_ERROR("Failed to set the number of poles of channel ", erotor->channel, ".");
      return;
    }
  }
  if (!transferAndSleep())
    return;

  // Load and set the speed control gains
  for (const auto& [link_name, _] : eprop->rotors)
  {
    const auto erotor = eprop->getRotor(link_name);
    if (erotor->channel >= aso::DShot::kChannelSize)
    {
      TOBAS_ERROR("Rotor channel ", erotor->channel, " is out of range.");
      continue;
    }
    if (!pt_.get(kGainKeyPrefix + to_string(erotor->channel), gains_.at(erotor->channel)))
    {
      TOBAS_ERROR("Failed to load the rotor speed control gain of channel ", erotor->channel, ".");
      continue;
    }
    if (!dshot_.setSpeedControlGain(erotor->channel, gains_.at(erotor->channel)))
    {
      TOBAS_ERROR("Failed to set the rotor speed control gain of channel ", erotor->channel, ".");
      continue;
    }
  }
  if (!transferAndSleep())
    return;

  // Resister publishers
  rotor_states_pub_ = createPublisher<tobas_msgs::msg::RotorStateArray>(tobas::kRotorStatesTopic);
  latency_pub_.initialize(shared_from_this());

  // Resister subscribers
  tar_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsCmdTopic, &self::targetSpeedsCb, this);

  // Resister service servers
  get_gains_ss_ = createService<GetGains>(tobas::kGetRotorControlGainsSrv, &self::getGainsCb, this);
  set_gains_ss_ = createService<SetGains>(tobas::kSetRotorControlGainsSrv, &self::setGainsCb, this);
  save_gains_ss_ = createService<SaveGains>(tobas::kSaveRotorControlGainsSrv, &self::saveGainsCb, this);

  // Create timers
  auto_stop_timer_ = createTimer(tobas::kCommandAutoResetTimeout, &self::autoStopTimerCb, this);

  eprop_ = eprop;
  TOBAS_INFO("Rotor speed controller is initialized.");
}

void DShotDriverNode::targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds)
{
  // Set target speeds of each channel
  for (const auto& tar_speed : tar_speeds->speeds)
  {
    const auto erotor = eprop_->getRotor(tar_speed.link_name);
    if (!erotor)
    {
      TOBAS_ERROR("Rotor \"" + tar_speed.link_name + "\" does not exist.");
      continue;
    }

    if (!dshot_.setTargetSpeed(erotor->channel, tar_speed.speed))
    {
      TOBAS_ERROR("Failed to set target speed of rotor \"", tar_speed.link_name, "\".");
      continue;
    }
  }

  // Send command and get rotor states
  if (!dshot_.transfer())
  {
    TOBAS_ERROR("SPI communication failed.");
    return;
  }

  // Publish messages
  publishRotorStates();
  latency_pub_.publish(tar_speeds->header.stamp);

  // Reset timeout timers
  auto_stop_timer_->reset();

  // Now the rotors are commanded
  is_commanded_ = true;
}

void DShotDriverNode::getGainsCb(const GetGains::Request::ConstSharedPtr&, const GetGains::Response::SharedPtr& res)
{
  res->gains.assign(gains_.begin(), gains_.end());
}

void DShotDriverNode::setGainsCb(const SetGains::Request::ConstSharedPtr& req, const SetGains::Response::SharedPtr& res)
{
  for (const auto& gain : req->gains)
  {
    if (!dshot_.setSpeedControlGain(gain.channel, gain.gain))
    {
      res->success = false;
      res->message = "Rotor control gain of channel " + to_string((int)gain.channel) + " is rejected.";
      return;
    }
    gains_.at(gain.channel) = gain.gain;
  }

  if (!dshot_.transfer())
  {
    res->success = false;
    res->message = "SPI communication with DShot driver is failed.";
    return;
  }

  res->success = true;
  res->message.clear();
}

void DShotDriverNode::saveGainsCb(const SaveGains::Request::ConstSharedPtr&, const SaveGains::Response::SharedPtr& res)
{
  for (size_t ch = 0; ch < aso::DShot::kChannelSize; ++ch)
  {
    const auto key = kGainKeyPrefix + to_string(ch);
    pt_.set(key, gains_.at(ch));
  }

  if (!pt_.save())
  {
    res->success = false;
    res->message = "Failed to save gains.";
    return;
  }

  res->success = true;
  res->message.clear();
}

void DShotDriverNode::autoStopTimerCb()
{
  if (!stopRotors())
    return;

  publishRotorStates();

  if (is_commanded_)
  {
    is_commanded_ = false;
    TOBAS_WARN(
      "All rotors are automatically stopped because ", tobas::kCommandAutoResetTimeout.count(),
      " ms have elapsed since the last command.");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(DShotDriverNode)
