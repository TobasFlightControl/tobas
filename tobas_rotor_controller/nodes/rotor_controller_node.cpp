#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs/msg/throttle_array.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/srv/enable_rc_output.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

using namespace std;

class RotorControllerNode : public tobas::BaseNode
{
  static constexpr double kWarnRate = 1.;  // [s]
  static constexpr double kDisarmThrottle = -0.1;
  static constexpr double kDisarmDuration = 3.;  // [s]
  static constexpr auto kDisarmInterval = 100ms;
  static constexpr auto kAutoStopTimeThresh = 100ms;  // アーム時は最低でも10Hzでスロットルを送る
  static constexpr auto kPublishArmingPeriod = 1s;

  using self = RotorControllerNode;
  using super = tobas::BaseNode;

public:
  explicit RotorControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool is_armed_ = false;
  bool is_activated_ = false;
  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  tobas_msgs::msg::PreArmCheck::ConstSharedPtr prearm_check_;

  // PubSub
  ros2::PublisherPtr<tobas_msgs::msg::ThrottleArray> throttles_pub_;
  ros2::PublisherPtr<std_msgs::msg::Bool> arming_pub_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeeds> tar_speeds_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;

  // Service
  ros2::ServiceServerPtr<tobas_msgs::srv::SetArm> set_arm_ss_;
  ros2::ServiceClientPtr<tobas_msgs::srv::EnableRCOutput> enable_rcout_sc_;

  // Timer
  ros2::TimerPtr publish_arming_timer_;
  ros2::TimerPtr auto_stop_timer_;

  bool armRotors();
  bool disarmRotors();
  bool enableRCOutputs(const bool& enable);
  void setThrottleOnAllChannels(const double& throttle);
  void publishArming();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void rotSpeedsCmdCb(const tobas_msgs::msg::RotorSpeeds::ConstSharedPtr& tar_speeds);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);

  void setArmCb(
    const tobas_msgs::srv::SetArm::Request::ConstSharedPtr& req,
    const tobas_msgs::srv::SetArm::Response::SharedPtr& res);

  void autoStopTimerCb();
};

RotorControllerNode::RotorControllerNode(const rclcpp::NodeOptions& options) : super("rotor_controller", options)
{
  throttles_pub_ = createPublisher<tobas_msgs::msg::ThrottleArray>(tobas::kThrottlesCmdTopic);
  arming_pub_ = createPublisher<std_msgs::msg::Bool>(tobas::kArmingTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tar_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsCmdTopic, &self::rotSpeedsCmdCb, this);
  battery_sub_ = createSubscriber(tobas::kBatteryLpfTopic, &self::batteryCb, this);
  prearm_check_sub_ = createSubscriber(tobas::kPreArmCheckTopic, &self::preArmCheckCb, this);

  set_arm_ss_ = createService<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv, &self::setArmCb, this);
  enable_rcout_sc_ = create_client<tobas_msgs::srv::EnableRCOutput>(tobas::kEnableRcOutputSrv);

  publish_arming_timer_ = createTimer(kPublishArmingPeriod, &self::publishArming, this);
  auto_stop_timer_ = createTimer(kAutoStopTimeThresh, &self::autoStopTimerCb, this, false);
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
  auto_stop_timer_->reset();

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
  auto_stop_timer_->cancel();

  return true;
}

bool RotorControllerNode::enableRCOutputs(const bool& enable)
{
  if (!enable_rcout_sc_->service_is_ready())
  {
    TOBAS_ERROR("\"", tobas::kEnableRcOutputSrv, "\" is not ready.");
    return false;
  }

  for (const auto& rotor : drone_->rotors)
  {
    const auto req = std::make_shared<tobas_msgs::srv::EnableRCOutput::Request>();
    req->channel = rotor.channel;
    req->enable = enable;

    enable_rcout_sc_->async_send_request(req);  // TODO: 結果を確認
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

  // Set a timer to reset the throttles if no command is received within a certain period of time
  auto_stop_timer_->reset();

  // Now the rotors are activated
  is_activated_ = true;
}

void RotorControllerNode::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

void RotorControllerNode::preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  prearm_check_ = prearm_check;
}

void RotorControllerNode::setArmCb(
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

    if (!armRotors())
    {
      res->success = false;
      res->message = "Failed to enable rotors.";
      TOBAS_ERROR(res->message);
      return;
    }
  }
  else if (is_armed_ && !req->arming)
  {
    if (!disarmRotors())
    {
      res->success = false;
      res->message = "Failed to disable rotors.";
      TOBAS_ERROR(res->message);
      return;
    }
  }

  publishArming();
  res->success = true;
}

void RotorControllerNode::autoStopTimerCb()
{
  setThrottleOnAllChannels(tobas::kMinThrot);
  if (is_activated_)
  {
    is_activated_ = false;
    TOBAS_WARN(
      "All rotors are automatically stopped because ", kAutoStopTimeThresh.count(),
      " ms have elapsed since the last command.");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorControllerNode)
