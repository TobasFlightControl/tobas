#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/core.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs/srv/get_rotor_control_gains.hpp>
#include <tobas_msgs/srv/set_rotor_control_gains.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

#include <tobas_aso_core/dshot.hpp>

class DShotDriverNode : public tobas::BaseNode
{
  using self = DShotDriverNode;
  using super = tobas::BaseNode;
  using SetArm = tobas_msgs::srv::SetArm;
  using GetGains = tobas_msgs::srv::GetRotorControlGains;
  using SetGains = tobas_msgs::srv::SetRotorControlGains;
  using SaveGains = std_srvs::srv::Trigger;

  static constexpr auto kSPIInterval = 1ms;
  static constexpr auto kAutoStopTimeThresh = 200ms;  // 最低でも5Hzでスロットルを送る
  static constexpr auto kAutoDisarmTimeThresh = 10s;
  static constexpr char kGainKeyPrefix[] = "speed_control_gain_";

public:
  explicit DShotDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::DShot dshot_;

  ptree::PropertyTree pt_;
  std::array<uint8_t, aso::DShot::kChannelSize> gains_ = { 0 };
  bool is_armed_ = false;
  bool is_commanded_ = false;
  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::PreArmCheck::ConstSharedPtr prearm_check_;

  ros2::PublisherPtr<tobas_msgs::msg::RotorStateArray> cur_states_pub_;
  ros2::PublisherPtr<std_msgs::msg::Bool> arming_pub_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;

  ros2::ServiceServerPtr<SetArm> set_arm_ss_;
  ros2::ServiceServerPtr<GetGains> get_gains_ss_;
  ros2::ServiceServerPtr<SetGains> set_gains_ss_;
  ros2::ServiceServerPtr<SaveGains> save_gains_ss_;

  ros2::TimerPtr publish_arm_status_timer_;
  ros2::TimerPtr auto_stop_timer_;
  ros2::TimerPtr auto_disarm_timer_;

  bool transferAndSleep();
  void publishCurrentStates();
  void publishArming();
  bool stopRotors();
  void arm();
  void disarm();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);

  void setArmCb(const SetArm::Request::ConstSharedPtr& req, const SetArm::Response::SharedPtr& res);
  void getGainsCb(const GetGains::Request::ConstSharedPtr& req, const GetGains::Response::SharedPtr& res);
  void setGainsCb(const SetGains::Request::ConstSharedPtr& req, const SetGains::Response::SharedPtr& res);
  void saveGainsCb(const SaveGains::Request::ConstSharedPtr& req, const SaveGains::Response::SharedPtr& res);

  void autoStopTimerCb();
  void autoDisarmTimerCb();
};

DShotDriverNode::DShotDriverNode(const rclcpp::NodeOptions& options) : super("aso_dshot_driver", options)
{
  if (!pt_.initialize((fs::path(real::kTobasResourceDir) / get_name()).replace_extension(".ini")))
  {
    TOBAS_ERROR("Failed to initialize property tree. This node will not work.");
    return;
  }

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);

  publish_arm_status_timer_ = createTimer(tobas::kPublishArmingPeriod, &self::publishArming, this, false);
  auto_stop_timer_ = createTimer(kAutoStopTimeThresh, &self::autoStopTimerCb, this, false);
  auto_disarm_timer_ = createTimer(kAutoDisarmTimeThresh, &self::autoDisarmTimerCb, this, false);
}

bool DShotDriverNode::transferAndSleep()
{
  if (!dshot_.transfer())
  {
    TOBAS_ERROR("SPI communication failed.");
    return false;
  }

  rclcpp::sleep_for(kSPIInterval);
  return true;
}

void DShotDriverNode::publishCurrentStates()
{
  auto cur_states = std::make_unique<tobas_msgs::msg::RotorStateArray>();
  cur_states->header.stamp = get_clock()->now();

  for (const auto& rotor : drone_->rotors)
  {
    cur_states->states.emplace_back();
    cur_states->states.back().channel = rotor.channel;
    cur_states->states.back().speed = dshot_.getSpeed(rotor.channel);
    cur_states->states.back().status = tobas_msgs::msg::RotorState::SPEED_ONLY;
  }

  cur_states_pub_->publish(move(cur_states));
}

void DShotDriverNode::publishArming()
{
  auto arming_msg = std::make_unique<std_msgs::msg::Bool>();
  arming_msg->data = is_armed_;
  arming_pub_->publish(move(arming_msg));
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

void DShotDriverNode::arm()
{
  is_armed_ = true;
  publishArming();
}

void DShotDriverNode::disarm()
{
  stopRotors();

  is_armed_ = false;
  publishArming();
}

void DShotDriverNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  if (drone_ != nullptr)
  {
    TOBAS_WARN("DShot driver cannot be re-initialized.");
    return;
  }

  // Initialize DShot driver
  if (!dshot_.initialize())
  {
    TOBAS_ERROR("Failed to initialize DShot driver.");
    return;
  }

  // Set Kv values
  for (const auto& rotor : drone->rotors)
  {
    if (!dshot_.setKv(rotor.channel, rotor.kv))
    {
      TOBAS_ERROR("Failed to set Kv of channel ", rotor.channel, ".");
      return;
    }
  }
  if (!transferAndSleep())
    return;

  // Set internal resistances
  for (const auto& rotor : drone->rotors)
  {
    if (!dshot_.setInternalResistance(rotor.channel, rotor.internal_resistance))
    {
      TOBAS_ERROR("Failed to set internal resistance of channel ", rotor.channel, ".");
      return;
    }
  }
  if (!transferAndSleep())
    return;

  // Set propeller diameters
  for (const auto& rotor : drone->rotors)
  {
    if (!dshot_.setPropellerDiameter(rotor.channel, rotor.propeller_diameter))
    {
      TOBAS_ERROR("Failed to set propeller diameter of channel ", rotor.channel, ".");
      return;
    }
  }
  if (!transferAndSleep())
    return;

  // Set moment constants
  for (const auto& rotor : drone->rotors)
  {
    const auto moment_const = rotor.motor_constant * rotor.moment_constant / math::quat(rotor.propeller_diameter);
    if (!dshot_.setMomentConstant(rotor.channel, moment_const))
    {
      TOBAS_ERROR("Failed to set moment constant of channel ", rotor.channel, ".");
      return;
    }
  }
  if (!transferAndSleep())
    return;

  // Set the number of poles
  for (const auto& rotor : drone->rotors)
  {
    if (!dshot_.setNumPoles(rotor.channel, rotor.num_poles))
    {
      TOBAS_ERROR("Failed to set the number of poles of channel ", rotor.channel, ".");
      return;
    }
  }
  if (!transferAndSleep())
    return;

  // Load and set the speed control gains
  for (const auto& rotor : drone->rotors)
  {
    if (rotor.channel >= aso::DShot::kChannelSize)
    {
      TOBAS_ERROR("Rotor channel ", rotor.channel, " is out of range.");
      continue;
    }
    if (!pt_.get(kGainKeyPrefix + std::to_string(rotor.channel), gains_.at(rotor.channel)))
    {
      TOBAS_ERROR("Failed to load the rotor speed control gain of channel ", rotor.channel, ".");
      continue;
    }
    if (!dshot_.setSpeedControlGain(rotor.channel, gains_.at(rotor.channel)))
    {
      TOBAS_ERROR("Failed to set the rotor speed control gain of channel ", rotor.channel, ".");
      continue;
    }
  }
  if (!transferAndSleep())
    return;

  // Resister publishers
  cur_states_pub_ = createPublisher<tobas_msgs::msg::RotorStateArray>(tobas::kRotorStatesTopic);
  arming_pub_ = createPublisher<std_msgs::msg::Bool>(tobas::kArmingTopic);

  // Resister subscribers
  tar_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsCmdTopic, &self::targetSpeedsCb, this);
  prearm_check_sub_ = createSubscriber(tobas::kPreArmCheckTopic, &self::preArmCheckCb, this);

  // Resister service servers
  set_arm_ss_ = createService<SetArm>(tobas::kSetArmSrv, &self::setArmCb, this);
  get_gains_ss_ = createService<GetGains>(tobas::kGetRotorControlGainsSrv, &self::getGainsCb, this);
  set_gains_ss_ = createService<SetGains>(tobas::kSetRotorControlGainsSrv, &self::setGainsCb, this);
  save_gains_ss_ = createService<SaveGains>(tobas::kSaveRotorControlGainsSrv, &self::saveGainsCb, this);

  // Start timers
  publish_arm_status_timer_->reset();
  auto_stop_timer_->reset();

  drone_ = drone;
  TOBAS_INFO("Rotor speed controller is initialized.");
}

void DShotDriverNode::targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds)
{
  if (!is_armed_)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Command is ignored because the rotors are disarmed.");
    return;
  }

  // Set target speeds of each channel
  for (const auto& tar_speed : tar_speeds->speeds)
  {
    if (tar_speed.channel >= aso::DShot::kChannelSize)
    {
      TOBAS_ERROR("DShot channel ", (int)tar_speed.channel, " does not exist.");
      return;
    }

    if (!dshot_.setTargetSpeed(tar_speed.channel, tar_speed.speed))
    {
      TOBAS_ERROR("Failed to set target speed on channel ", (int)tar_speed.channel, ".");
      return;
    }
  }

  // Send command and get current states
  if (!dshot_.transfer())
  {
    TOBAS_ERROR("SPI communication failed.");
    return;
  }

  // Publish current speeds
  publishCurrentStates();

  // Reset timeout timers
  auto_stop_timer_->reset();
  auto_disarm_timer_->reset();

  // Now the rotors are commanded
  is_commanded_ = true;
}

void DShotDriverNode::preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  prearm_check_ = prearm_check;
}

void DShotDriverNode::setArmCb(const SetArm::Request::ConstSharedPtr& req, const SetArm::Response::SharedPtr& res)
{
  if (!is_armed_ && req->arming)
  {
    if (!req->ignore_prearm_check)
    {
      if (prearm_check_ == nullptr)
      {
        res->success = false;
        res->message = "Pre-arm check status is not received yet.";
        return;
      }

      if (!prearm_check_->ok)
      {
        res->success = false;
        res->message = "Pre-arm check failed.";
        return;
      }
    }

    arm();
    auto_disarm_timer_->reset();
  }
  else if (is_armed_ && !req->arming)
  {
    disarm();
    auto_disarm_timer_->cancel();
  }

  res->success = true;
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
    const auto key = kGainKeyPrefix + std::to_string(ch);
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

  publishCurrentStates();

  if (is_commanded_)
  {
    is_commanded_ = false;
    TOBAS_WARN(
      "All rotors are automatically stopped because ", kAutoStopTimeThresh.count(),
      " ms have elapsed since the last command.");
  }
}

void DShotDriverNode::autoDisarmTimerCb()
{
  disarm();
  auto_disarm_timer_->cancel();

  TOBAS_WARN(
    "All rotors are automatically disarmed because ", kAutoDisarmTimeThresh.count(),
    " s have elapsed since the last command.");
}

RCLCPP_COMPONENTS_REGISTER_NODE(DShotDriverNode)
