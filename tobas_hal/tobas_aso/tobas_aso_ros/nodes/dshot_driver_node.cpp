#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/srv/enable_rc_output.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

#include <tobas_aso_core/dshot.hpp>

#define SPEED_CONTROL_GAIN 16  // TODO: ユーザが調整できるように

class DShotDriverNode : public tobas::BaseNode
{
  using self = DShotDriverNode;
  using super = tobas::BaseNode;
  using EnableSrv = tobas_msgs::srv::EnableRCOutput;

  static constexpr auto kSPIInterval = 1ms;
  static constexpr auto kAutoStopTimeThresh = 200ms;  // 最低でも5Hzでスロットルを送る

public:
  explicit DShotDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::DShot dshot_;
  bool is_activated_ = false;
  array<bool, aso::DShot::kChannelSize> is_enabled_;

  tobas::Drone::ConstSharedPtr drone_;

  ros2::PublisherPtr<tobas_msgs::msg::RotorSpeedArray> cur_speeds_pub_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_sub_;

  ros2::ServiceServerPtr<EnableSrv> enable_rcout_srv_;

  ros2::TimerPtr auto_stop_timer_;

  bool transferAndSleep();
  void publishCurrentSpeeds();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds);
  void enableRCOutputCb(const EnableSrv::Request::ConstSharedPtr& req, const EnableSrv::Response::SharedPtr& res);

  void autoStopTimerCb();
};

DShotDriverNode::DShotDriverNode(const rclcpp::NodeOptions& options) : super("aso_dshot_driver", options)
{
  cur_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeedArray>(tobas::kRotorSpeedsTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tar_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsCmdTopic, &self::targetSpeedsCb, this);

  enable_rcout_srv_ = createService<EnableSrv>(tobas::kEnableRcOutputSrv, &self::enableRCOutputCb, this);

  auto_stop_timer_ = createTimer(kAutoStopTimeThresh, &self::autoStopTimerCb, this, false);
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

void DShotDriverNode::publishCurrentSpeeds()
{
  auto cur_speeds = std::make_unique<tobas_msgs::msg::RotorSpeedArray>();
  cur_speeds->header.stamp = get_clock()->now();

  for (const auto& rotor : drone_->rotors)
  {
    cur_speeds->speeds.emplace_back();
    cur_speeds->speeds.back().channel = rotor.channel;
    cur_speeds->speeds.back().speed = dshot_.getSpeed(rotor.channel);
  }

  cur_speeds_pub_->publish(move(cur_speeds));
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

  // Set speed control gains
  for (const auto& rotor : drone->rotors)
  {
    if (!dshot_.setSpeedControlGain(rotor.channel, SPEED_CONTROL_GAIN))
    {
      TOBAS_ERROR("Failed to set the speed control gain of channel ", rotor.channel, ".");
      return;
    }
  }
  if (!transferAndSleep())
    return;

  drone_ = drone;
  TOBAS_INFO("Rotor speed controller is initialized.");
}

void DShotDriverNode::targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds)
{
  if (drone_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Drone configuration is not received yet.");
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

    if (!is_enabled_.at(tar_speed.channel))
    {
      TOBAS_ERROR("DShot channel ", (int)tar_speed.channel, " is disabled.");
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
  publishCurrentSpeeds();

  // Set a timer to reset the rotor speeds if no command is received within a certain period of time
  auto_stop_timer_->reset();

  // Now the rotors are activated
  is_activated_ = true;
}

void DShotDriverNode::enableRCOutputCb(
  const EnableSrv::Request::ConstSharedPtr& req,
  const EnableSrv::Response::SharedPtr& res)
{
  if (req->channel >= aso::DShot::kChannelSize)
  {
    res->success = false;
    res->message = "DShot channel out of range.";
    return;
  }

  if (req->enable)
  {
    is_enabled_.at(req->channel) = true;
  }
  else
  {
    dshot_.setThrottle(req->channel, aso::DShot::DSHOT_CMD_MOTOR_STOP);
    if (!dshot_.transfer())
    {
      res->success = false;
      res->message = "Failed to send command.";
    }
    is_enabled_.at(req->channel) = false;
  }

  res->success = true;
  res->message.clear();
}

void DShotDriverNode::autoStopTimerCb()
{
  for (size_t ch = 0; ch < aso::DShot::kChannelSize; ++ch)
    dshot_.setThrottle(ch, aso::DShot::DSHOT_CMD_MOTOR_STOP);

  if (!dshot_.transfer())
  {
    TOBAS_ERROR("Failed to stop motors.");
    return;
  }

  if (is_activated_)
  {
    is_activated_ = false;
    TOBAS_WARN(
      "All rotors are automatically stopped because ", kAutoStopTimeThresh.count(),
      " ms have elapsed since the last command.");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(DShotDriverNode)
