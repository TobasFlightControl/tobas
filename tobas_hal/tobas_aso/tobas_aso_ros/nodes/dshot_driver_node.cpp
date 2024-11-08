#include <std_msgs/msg/bool.hpp>

#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

#include <tobas_aso_core/dshot.hpp>

class DShotDriverNode : public tobas::BaseNode
{
  using self = DShotDriverNode;
  using super = tobas::BaseNode;

  static constexpr auto kSPIInterval = 1ms;
  static constexpr auto kAutoStopTimeThresh = 200ms;  // 最低でも5Hzでスロットルを送る

public:
  explicit DShotDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::DShot dshot_;

  bool is_armed_ = false;
  bool is_commanded_ = false;
  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::PreArmCheck::ConstSharedPtr prearm_check_;

  ros2::PublisherPtr<tobas_msgs::msg::RotorSpeedArray> cur_speeds_pub_;
  ros2::PublisherPtr<std_msgs::msg::Bool> arming_pub_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;

  ros2::ServiceServerPtr<tobas_msgs::srv::SetArm> set_arm_ss_;

  ros2::TimerPtr publish_arm_status_timer_;
  ros2::TimerPtr auto_stop_timer_;

  bool transferAndSleep();
  void publishCurrentSpeeds();
  void publishArming();
  bool stopRotors();

  template <size_t N>
  void addGainParams();

  template <size_t Channel>
  bool controlGainCb(const long& p);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);

  void setArmCb(
    const tobas_msgs::srv::SetArm::Request::ConstSharedPtr& req,
    const tobas_msgs::srv::SetArm::Response::SharedPtr& res);

  void autoStopTimerCb();
};

DShotDriverNode::DShotDriverNode(const rclcpp::NodeOptions& options) : super("aso_dshot_driver", options)
{
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
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

template <size_t N>
void DShotDriverNode::addGainParams()
{
  if constexpr (N > 0)
  {
    const auto name = tobas::kRotorControlGainParamPrefix + to_string(N - 1);
    addDynamicIntParam(name, &self::controlGainCb<N - 1>, this, 0, tobas::kMinRotorCtrlGain, tobas::kMaxRotorCtrlGain);

    addGainParams<N - 1>();
  }
}

template <size_t Channel>
bool DShotDriverNode::controlGainCb(const long& p)
{
  if (!dshot_.setSpeedControlGain(Channel, p))
  {
    TOBAS_ERROR("Failed to set control gain on channel ", Channel, ".");
    return false;
  }

  if (!dshot_.transfer())
  {
    TOBAS_ERROR("SPI communication failed.");
    return false;
  }

  return true;
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

  // Register dynamic parameters
  addGainParams<aso::DShot::kChannelSize>();  // コンパイル時に全チャンネルの登録操作を展開

  // Resister publishers
  cur_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeedArray>(tobas::kRotorSpeedsTopic);
  arming_pub_ = createPublisher<std_msgs::msg::Bool>(tobas::kArmingTopic);

  // Resister subscribers
  tar_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsCmdTopic, &self::targetSpeedsCb, this);
  prearm_check_sub_ = createSubscriber(tobas::kPreArmCheckTopic, &self::preArmCheckCb, this);

  // Resister service servers
  set_arm_ss_ = createService<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv, &self::setArmCb, this);

  // Start timers
  publish_arm_status_timer_ = createTimer(tobas::kPublishArmingPeriod, &self::publishArming, this);
  auto_stop_timer_ = createTimer(kAutoStopTimeThresh, &self::autoStopTimerCb, this);

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
  publishCurrentSpeeds();

  // Set a timer to reset the rotor speeds if no command is received within a certain period of time
  auto_stop_timer_->reset();

  // Now the rotors are commanded
  is_commanded_ = true;
}

void DShotDriverNode::preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  prearm_check_ = prearm_check;
}

void DShotDriverNode::setArmCb(
  const tobas_msgs::srv::SetArm::Request::ConstSharedPtr& req,
  const tobas_msgs::srv::SetArm::Response::SharedPtr& res)
{
  TOBAS_INFO("Set arm requested.");

  if (!is_armed_ && req->arming)
  {
    if (!req->ignore_prearm_check)
    {
      if (prearm_check_ == nullptr)
      {
        res->success = false;
        res->message = "Pre-arm check status is not received yet.";
        TOBAS_ERROR(res->message);
        return;
      }

      if (!prearm_check_->ok)
      {
        res->success = false;
        res->message = "Pre-arm check failed.";
        TOBAS_ERROR(res->message);
        return;
      }
    }

    is_armed_ = true;
    publishArming();
  }
  else if (is_armed_ && !req->arming)
  {
    stopRotors();
    is_armed_ = false;
    publishArming();
  }

  res->success = true;
}

void DShotDriverNode::autoStopTimerCb()
{
  if (!stopRotors())
    return;

  if (is_commanded_)
  {
    is_commanded_ = false;
    TOBAS_WARN(
      "All rotors are automatically stopped because ", kAutoStopTimeThresh.count(),
      " ms have elapsed since the last command.");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(DShotDriverNode)
