#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_ros2_tools/simple_service_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs/msg/throttle_array.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/srv/enable_rc_output.hpp>
#include <tobas_msgs/srv/get_arm.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_drone_msgs/Drone.hpp>

using namespace std;

class RotorControllerNode : public tobas::BaseNode
{
  static constexpr double kWarnRate = 1.;  // [s]
  static constexpr double kDisarmThrottle = -0.1;
  static constexpr double kDisarmDuration = 3.;  // [s]
  static constexpr auto kDisarmInterval = 100ms;
  static constexpr auto kCheckIntervalTimerPeriod = 100ms;

  using self = RotorControllerNode;
  using super = tobas::BaseNode;

  using GetSrv = tobas_msgs::srv::GetArm;
  using SetSrv = tobas_msgs::srv::SetArm;

public:
  explicit RotorControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::Time last_cmd_time_;
  bool is_armed_ = false;
  bool is_activated_ = false;
  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;

  // PubSub
  PublisherPtr<tobas_msgs::msg::ThrottleArray> throttles_pub_;
  PublisherPtr<std_msgs::msg::Bool> arming_pub_;
  SubscriberPtr<tobas::Drone> drone_sub_;
  SubscriberPtr<tobas_msgs::msg::RotorSpeeds> tar_speeds_sub_;
  SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;

  // Service
  ServicePtr<GetSrv> get_arm_ss_;
  ServicePtr<SetSrv> set_arm_ss_;

  // Timer
  TimerPtr check_interval_timer_;

  bool armRotors();
  bool disarmRotors();
  bool enableRCOutputs(const bool& enable);
  bool preArmCheck();
  void setThrottleOnAllChannels(const double& throttle);
  void publishArming();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void rotSpeedsCmdCb(const tobas_msgs::msg::RotorSpeeds::ConstSharedPtr& tar_speeds);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);

  void getArmCb(const GetSrv::Request::ConstSharedPtr& req, const GetSrv::Response::SharedPtr& res);
  void setArmCb(const SetSrv::Request::ConstSharedPtr& req, const SetSrv::Response::SharedPtr& res);

  void checkIntervalTimerCb();
};

RotorControllerNode::RotorControllerNode(const rclcpp::NodeOptions& options) : super("rotor_controller", options)
{
  throttles_pub_ = createPublisher<tobas_msgs::msg::ThrottleArray>(tobas::kThrottlesCmdTopic);
  arming_pub_ = createPublisher<std_msgs::msg::Bool>(tobas::kArmingTopic, 1, true);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this);
  tar_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsCmdTopic, &self::rotSpeedsCmdCb, this);
  battery_sub_ = createSubscriber(tobas::kBatteryLpfTopic, &self::batteryCb, this);

  get_arm_ss_ = createService<GetSrv>(tobas::kGetArmSrv, &self::getArmCb, this);
  set_arm_ss_ = createService<SetSrv>(tobas::kSetArmSrv, &self::setArmCb, this);

  check_interval_timer_ = createTimer(kCheckIntervalTimerPeriod, &self::checkIntervalTimerCb, this);
  check_interval_timer_->cancel();

  publishArming();
}

bool RotorControllerNode::armRotors()
{
  if (!enableRCOutputs(true))
  {
    TOBAS_ERROR("Failed to enable rotors.");
    return false;
  }

  const auto t_start = get_clock()->now();
  while ((get_clock()->now() - t_start).seconds() < kDisarmDuration)
  {
    setThrottleOnAllChannels(kDisarmThrottle);
    rclcpp::sleep_for(kDisarmInterval);
  }

  is_armed_ = true;
  check_interval_timer_->reset();

  TOBAS_INFO("Rotors are ready to rotate.");
  return true;
}

bool RotorControllerNode::disarmRotors()
{
  if (!enableRCOutputs(false))
  {
    TOBAS_ERROR("Failed to disable rotors.");
    return false;
  }

  is_armed_ = false;
  check_interval_timer_->cancel();

  return true;
}

bool RotorControllerNode::enableRCOutputs(const bool& enable)
{
  ros2::SimpleServiceClient<tobas_msgs::srv::EnableRCOutput> sc(shared_from_this(), tobas::kEnableRcOutputSrv);

  for (const auto& rotor : drone_->rotors)
  {
    const auto req = std::make_shared<tobas_msgs::srv::EnableRCOutput::Request>();
    req->channel = rotor.channel;
    req->enable = enable;
    if (!sc.call(req))
    {
      TOBAS_ERROR("Failed to call \"", tobas::kEnableRcOutputSrv, "\" service.");
      return false;
    }

    const auto& res = sc.getResponse();
    if (!res->success)
    {
      TOBAS_ERROR("Failed to enable RC output: ", res->message);
      return false;
    }
  }

  return true;
}

bool RotorControllerNode::preArmCheck()
{
  ros2::SimpleServiceClient<std_srvs::srv::Trigger> sc(shared_from_this(), tobas::kPreArmCheckSrv);

  const auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  if (!sc.call(req))
  {
    TOBAS_ERROR("Failed to call \"", tobas::kPreArmCheckSrv, "\" service.");
    return false;
  }

  const auto& res = sc.getResponse();
  if (!res->success)
  {
    TOBAS_ERROR("Pre-arm check failed: ", res->message);
    return false;
  }

  return true;
}

void RotorControllerNode::setThrottleOnAllChannels(const double& throttle)
{
  auto throttles = std::make_unique<tobas_msgs::msg::ThrottleArray>();
  throttles->header.stamp = get_clock()->now();
  for (const auto& rotor : drone_->rotors)
  {
    throttles->throttles.emplace_back();
    throttles->throttles.back().channel = rotor.channel;
    throttles->throttles.back().throttle = throttle;
  }
  throttles_pub_->publish(move(throttles));
}

void RotorControllerNode::publishArming()
{
  auto arming_msg = std::make_unique<std_msgs::msg::Bool>();
  arming_msg->data = is_armed_;
  arming_pub_->publish(move(arming_msg));
}

void RotorControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
}

void RotorControllerNode::rotSpeedsCmdCb(const tobas_msgs::msg::RotorSpeeds::ConstSharedPtr& tar_speeds)
{
  if (!is_armed_)
    return;

  if (drone_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(kWarnRate, "Rotors cannot be rotated because drone configuration has not been received yet.");
    return;
  }

  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(kWarnRate, "Rotors cannot be rotated because battery state has not been received yet.");
    return;
  }

  const auto data_size = tar_speeds->speeds.size();
  if (data_size != drone_->numRotors())
  {
    TOBAS_ERROR("Size mismatch: ", data_size, " != ", drone_->numRotors());
    return;
  }

  // Create throttle message
  auto throttles = std::make_unique<tobas_msgs::msg::ThrottleArray>();
  throttles->header = tar_speeds->header;

  // Update throttles
  for (size_t rotor_idx = 0; rotor_idx < data_size; ++rotor_idx)
  {
    const auto& rotor = drone_->rotors.at(rotor_idx);

    // 目標回転数を決定
    const auto max_speed = drone_->maxRotSpeed(rotor_idx, battery_->voltage);
    auto tar_speed = tar_speeds->speeds[rotor_idx];
    if (tar_speed < 0.)  // モータテストでも使用するため，ここではARM_THROTTLEの制約を課さない
    {
      TOBAS_WARN("Negative rotation speed is commanded on CH", rotor_idx, ": ", tar_speed, " < 0 [rad/s]");
      tar_speed = 0.;
    }
    else if (tar_speed > max_speed + tobas::kRotSpeedMargin)
    {
      TOBAS_WARN("Target rotation speed of CH", rotor_idx, " is too high: ", tar_speed, " > ", max_speed, " [rad/s]");
      tar_speed = max_speed;
    }

    // 目標スロットルを決定
    double throt;
    switch (rotor.esc_mode)
    {
      case tobas::BLHELI_OPEN_LOOP:
      {
        throt = drone_->throttleFromRotSpeed(rotor_idx, tar_speed, battery_->voltage);
        break;
      }
      case tobas::BLHELI_CLOSED_LOOP_LOW_RANGE:
      {
        const auto erpm = drone_->erpmFromRotSpeed(rotor_idx, tar_speed);
        throt = math::remap(erpm, 0., tobas::esc::kBLHeliCLLowMaxERPM, tobas::kMinThrot, tobas::kMaxThrot);
        break;
      }
      case tobas::BLHELI_CLOSED_LOOP_MID_RANGE:
      {
        const auto erpm = drone_->erpmFromRotSpeed(rotor_idx, tar_speed);
        throt = math::remap(erpm, 0., tobas::esc::kBLHeliCLMidMaxERPM, tobas::kMinThrot, tobas::kMaxThrot);
        break;
      }
      case tobas::BLHELI_CLOSED_LOOP_HIGH_RANGE:
      {
        const auto erpm = drone_->erpmFromRotSpeed(rotor_idx, tar_speed);
        throt = math::remap(erpm, 0., tobas::esc::kBLHeliCLHighMaxERPM, tobas::kMinThrot, tobas::kMaxThrot);
        break;
      }
      default:
      {
        TOBAS_ERROR("Unknown ESC signal mode of CH", rotor.channel);
        throt = tobas::kMinThrot;
        break;
      }
    }

    // Add throttle command
    throttles->throttles.emplace_back();
    throttles->throttles.back().channel = rotor.channel;
    throttles->throttles.back().throttle = throt;
  }

  // Publish throttle commands
  throttles_pub_->publish(move(throttles));

  // Update last commanded time
  last_cmd_time_ = get_clock()->now();

  // Now the rotors are activated
  is_activated_ = true;
}

void RotorControllerNode::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

void RotorControllerNode::getArmCb(const GetSrv::Request::ConstSharedPtr&, const GetSrv::Response::SharedPtr& res)
{
  res->arming = is_armed_;
}

void RotorControllerNode::setArmCb(const SetSrv::Request::ConstSharedPtr& req, const SetSrv::Response::SharedPtr& res)
{
  if (!is_armed_ && req->arming)
  {
    if (!req->ignore_prearm_check && !preArmCheck())
    {
      res->success = false;
      res->message = "Pre-arm check failed.";
      return;
    }

    if (!armRotors())
    {
      res->success = false;
      res->message = "Failed to enable rotors.";
      return;
    }
  }
  else if (is_armed_ && !req->arming)
  {
    if (!disarmRotors())
    {
      res->success = false;
      res->message = "Failed to disable rotors.";
      return;
    }
  }

  publishArming();
  res->success = true;
}

void RotorControllerNode::checkIntervalTimerCb()
{
  const auto time_after_last_cmd = (get_clock()->now() - last_cmd_time_).seconds();
  if (time_after_last_cmd > tobas::kAutoResetTimeThreshold)
  {
    setThrottleOnAllChannels(tobas::kMinThrot);
    if (is_activated_)
    {
      is_activated_ = false;
      TOBAS_WARN(
        "The speeds of all rotors are automatically stopped because ", tobas::kAutoResetTimeThreshold,
        " seconds have elapsed since the last command.");
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorControllerNode)
