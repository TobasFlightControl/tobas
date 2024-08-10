#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/vector.hpp>

#include <tobas_constants/constants.hpp>

#include <tobas_msgs/msg/throttle_array.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>
#include <tobas_msgs/srv/enable_rc_output.hpp>

#include "../include/tobas_rotor_controller/rotor_controller.hpp"

using namespace std;

namespace tobas_rotor_controller
{
RotorController::RotorController(const rclcpp::NodeOptions& options) : super(node, pnh, name)
{


  throttles_pub_ = createPublisher<tobas_msgs::msg::ThrottleArray>(tobas::kThrottlesCmdTopic);
  arming_pub_ = createPublisher<std_msgs::msg::Bool>(tobas::kArmingTopic, 1, true);

  tar_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsCmdTopic, &self::rotSpeedsCmdCb, this);
  battery_sub_ = createSubscriber(tobas::kBatteryLpfTopic, &self::batteryCb, this);

  get_arm_ss_ = createService(tobas::kGetArmSrv, &self::getArmCb, this);
  set_arm_ss_ = createService(tobas::kSetArmSrv, &self::setArmCb, this);
  enable_rcout_sc_ = node_.serviceClient<tobas_msgs::EnableRCOutput>(tobas::kEnableRcOutputSrv);
  pre_arm_check_sc_ = node_.serviceClient<std_srvs::Trigger>(tobas::kPreArmCheckSrv);

  check_interval_timer_ = node_.createTimer(kCheckIntervalTimerRate, &self::checkIntervalTimerCb, this, false, false);

  publishArming();
}

bool RotorController::armRotors()
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
    rclcpp::Duration(kDisarmInterval).sleep();
  }

  is_armed_ = true;
  check_interval_timer_.start();

  TOBAS_INFO("The  are ready to rotate.");
  return true;
}

bool RotorController::disarmRotors()
{
  if (!enableRCOutputs(false))
  {
    TOBAS_ERROR("Failed to disable rotors.");
    return false;
  }

  is_armed_ = false;
  check_interval_timer_.stop();

  return true;
}

bool RotorController::enableRCOutputs(const bool& enable)
{
  if (!enable_rcout_sc_.wait_for_service(rclcpp::Duration(tobas::kWaitForServiceExistence)))
  {
    TOBAS_ERROR("Failed to connect to '", tobas::kEnableRcOutputSrv, "' server.");
    return false;
  }

  tobas_msgs::EnableRCOutput enable_rcout_msg;
  for (const auto& rotor : drone_.rotors)
  {
    enable_rcout_msg.request.channel = rotor.channel;
    enable_rcout_msg.request.enable = enable;
    if (!enable_rcout_sc_.call(enable_rcout_msg) || !enable_rcout_msg.response.success)
    {
      TOBAS_ERROR(enable_rcout_msg.response.message);
      return false;
    }
  }

  return true;
}

bool RotorController::preArmCheck()
{
  if (!pre_arm_check_sc_.wait_for_service(rclcpp::Duration(tobas::kWaitForServiceExistence)))
  {
    TOBAS_ERROR("Failed to connect to '", tobas::kPreArmCheckSrv, "' server.");
    return false;
  }

  std_srvs::Trigger pre_arm_check_msg;
  if (!pre_arm_check_sc_.call(pre_arm_check_msg) || !pre_arm_check_msg.response.success)
  {
    TOBAS_ERROR(pre_arm_check_msg.response.message);
    return false;
  }

  return true;
}

void RotorController::setThrottleOnAllChannels(const double& throttle)
{
  const auto throttles =std::make_unique<tobas_msgs::msg::ThrottleArray>();
  throttles->header.stamp = get_clock()->now();
  for (const auto& rotor : drone_.rotors)
    throttles->throttles.emplace_back(rotor.channel, throttle);
  throttles_pub_->publish(throttles);
}

void RotorController::publishArming()
{
  const auto arming_msg =std::make_unique<std_msgs::msg::Bool>();
  arming_msg->data = is_armed_;
  arming_pub_->publish(arming_msg);
}

void RotorController::rotSpeedsCmdCb(const tobas_msgs::msg::RotorSpeeds::ConstSharedPtr& tar_speeds)
{
  if (!is_armed_)
    return;

  if (battery_ == nullptr)
  {
    TOBAS_ERROR("The rotors cannot be rotated because battery state has not been received yet.");
    return;
  }

  const auto data_size = tar_speeds->speeds.size();
  if (data_size != drone_.numRotors())
  {
    TOBAS_ERROR("Size mismatch: ", data_size, " != ", drone_.numRotors());
    return;
  }

  // Create throttle message
  const auto throttles =std::make_unique<tobas_msgs::msg::ThrottleArray>();
  throttles->header = tar_speeds->header;

  // Update throttles
  for (size_t rotor_idx = 0; rotor_idx < data_size; ++rotor_idx)
  {
    const auto& rotor = drone_.rotorConfig(rotor_idx);

    // 目標回転数を決定
    const auto max_speed = drone_.maxRotSpeed(rotor_idx, battery_->voltage);
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

    // スロットルコマンドメッセージを作成
    double throttle;
    switch (rotor.esc_mode.value)
    {
      case tobas::BLHELI_OPEN_LOOP.value:
      {
        throttle = drone_.throttleFromRotSpeed(rotor_idx, tar_speed, battery_->voltage);
        break;
      }
      case tobas::BLHELI_CLOSED_LOOP_LOW_RANGE.value:
      {
        const auto erpm = drone_.erpmFromRotSpeed(rotor_idx, tar_speed);
        throttle = math::remap(erpm, 0., tobas::kBLHeliCLLowMaxERPM, tobas::kMinThrottle, tobas::kMaxThrottle);
        break;
      }
      case tobas::BLHELI_CLOSED_LOOP_MID_RANGE.value:
      {
        const auto erpm = drone_.erpmFromRotSpeed(rotor_idx, tar_speed);
        throttle = math::remap(erpm, 0., tobas::kBLHeliCLMidMaxERPM, tobas::kMinThrottle, tobas::kMaxThrottle);
        break;
      }
      case tobas::BLHELI_CLOSED_LOOP_HIGH_RANGE.value:
      {
        const auto erpm = drone_.erpmFromRotSpeed(rotor_idx, tar_speed);
        throttle = math::remap(erpm, 0., tobas::kBLHeliCLHighMaxERPM, tobas::kMinThrottle, tobas::kMaxThrottle);
        break;
      }
      default:
      {
        TOBAS_ERROR("Unknown ESC signal mode of CH", rotor.channel);
        throttle = tobas::kMinThrottle;
        break;
      }
    }
    throttles->throttles.emplace_back(rotor.channel, throttle);
  }

  // Publish throttle commands
  throttles_pub_->publish(throttles);

  // Update last commanded time
  last_cmd_time_ = get_clock()->now();

  // Now the rotors are activated
  is_activated_ = true;
}

void RotorController::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

bool RotorController::getArmCb(tobas_msgs::GetArmRequest&, tobas_msgs::GetArmResponse& res)
{
  res.arming = is_armed_;
  return true;
}

bool RotorController::setArmCb(tobas_msgs::SetArmRequest& req, tobas_msgs::SetArmResponse& res)
{
  if (!is_armed_ && req.arming)
  {
    if (!req.ignore_pre_arm_check && !preArmCheck())
    {
      res.success = false;
      res.message = "Pre-arm check failed.";
      return true;
    }

    if (!armRotors())
    {
      res.success = false;
      res.message = "Failed to enable rotors.";
      return true;
    }
  }
  else if (is_armed_ && !req.arming)
  {
    if (!disarmRotors())
    {
      res.success = false;
      res.message = "Failed to disable rotors.";
      return true;
    }
  }

  publishArming();

  res.success = true;
  return true;
}

void RotorController::checkIntervalTimerCb(const rclcpp::TimerEvent& event)
{
  const auto time_after_last_cmd = (event.current_real - last_cmd_time_).seconds();
  if (time_after_last_cmd > tobas::kAutoResetTimeThreshold)
  {
    setThrottleOnAllChannels(tobas::kMinThrottle);
    if (is_activated_)
    {
      is_activated_ = false;
      TOBAS_WARN(
        "The speeds of all rotors are automatically stopped because ", tobas::kAutoResetTimeThreshold,
        " seconds have elapsed since the last command.");
    }
  }
}
}  // namespace tobas_rotor_controller
