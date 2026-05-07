// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_fc2xx_core/pwm_batt_imu.hpp>
#include <tobas_node/node.hpp>
#include <tobas_real_common/ros_interface.hpp>
#include <tobas_tools/imu_sampling_time_publisher.hpp>

#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>
#include <tobas_msgs/srv/configure_imu_filter.hpp>
#include <tobas_msgs_adapter/imu.hpp>

#include "./common.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace fc2xx
{
class PwmBattImuDriverNode : public BaseNode
{
  using self = PwmBattImuDriverNode;
  using super = BaseNode;

  static constexpr auto kSamplingPeriod = 1250us;  // 800Hz

public:
  explicit PwmBattImuDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PwmBattImu driver_;
  uint16_t pwm_periods_[PwmBattImu::kPwmChannels] = {};

  ros2::PublisherPtr<tobas_msgs::msg::Battery> batt_pub_;
  ros2::PublisherPtr<tobas_msgs::Imu> imu_raw_pub_;
  ros2::PublisherPtr<tobas_msgs::Imu> imu_filt_pub_;
  ImuSamplingTimePublisher sampling_time_pub_;

  ros2::SubscriberPtr<tobas_msgs::msg::PwmArray> pwms_sub_;

  ros2::ServiceServerPtr<tobas_msgs::srv::ConfigureImuFilter> config_ss_;

  ros2::TimerPtr initialize_timer_, main_timer_;

  void initialize();

  void pwmsCb(const tobas_msgs::msg::PwmArray::ConstSharedPtr& pwms);

  void configureImuFilterCb(
    const tobas_msgs::srv::ConfigureImuFilter::Request::ConstSharedPtr& req,
    const tobas_msgs::srv::ConfigureImuFilter::Response::SharedPtr& res);

  void mainTimerCb();
};

PwmBattImuDriverNode::PwmBattImuDriverNode(const rclcpp::NodeOptions& options)
  : super("fc2xx_pwm_batt_imu_driver", nodeOptions_Default(options))
{
  initialize_timer_ = createWallTimer(kRetryInitializationInterval, &self::initialize, this);
}

void PwmBattImuDriverNode::initialize()
{
  if (!driver_.initialize()) {
    TOBAS_ERROR("Failed to initialize the driver. Retrying...");
    return;
  }

  batt_pub_ = createPublisher<tobas_msgs::msg::Battery>(topic::kBattery);
  imu_raw_pub_ = createPublisher<tobas_msgs::Imu>(real::topic::kImuRaw);
  imu_filt_pub_ = createPublisher<tobas_msgs::Imu>(real::topic::kImuFilt);
  sampling_time_pub_.initialize(shared_from_this(), now());

  pwms_sub_ = createSubscriber(topic::kPwmCmd, &self::pwmsCb, this);

  config_ss_ =
    createService<tobas_msgs::srv::ConfigureImuFilter>(service::kConfigureImuFilter, &self::configureImuFilterCb, this);

  initialize_timer_->cancel();
  main_timer_ = createWallTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void PwmBattImuDriverNode::pwmsCb(const tobas_msgs::msg::PwmArray::ConstSharedPtr& pwms)
{
  for (const auto& elem : pwms->pwms) {
    if (elem.channel >= PwmBattImu::kPwmChannels) {
      TOBAS_ERROR("PWM channel ", elem.channel, " is out of range.");
      continue;
    }
    pwm_periods_[elem.channel] = elem.period;
  }

  driver_.setPwmPeriod(pwm_periods_);
}

void PwmBattImuDriverNode::configureImuFilterCb(
  const tobas_msgs::srv::ConfigureImuFilter::Request::ConstSharedPtr& req,
  const tobas_msgs::srv::ConfigureImuFilter::Response::SharedPtr& res)
{
  driver_.setImuLpfCutoff(req->accel_cutoff, req->gyro_cutoff, req->dgyro_cutoff);

  if (!driver_.transfer()) {
    res->success = false;
    res->message = "Failed to communicate with the MCU.";
    return;
  }

  res->success = true;
  res->message.clear();
}

void PwmBattImuDriverNode::mainTimerCb()
{
  // Get the current time
  const auto cur_time = now();

  // Communicate with the MCU
  if (!driver_.transfer()) {
    TOBAS_ERROR("Failed to communicate with the MCU.");
    return;
  }

  // Publish the battery state
  auto batt_msg = std::make_unique<tobas_msgs::msg::Battery>();
  batt_msg->header.stamp = cur_time;
  driver_.getBattVoltage(batt_msg->voltage);
  driver_.getBattCurrent(batt_msg->current);
  if (batt_msg->voltage > 0. && batt_msg->current > 0.) {
    batt_pub_->publish(std::move(batt_msg));
  }
  else {
    TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Battery state is unavailable.");
  }

  // Publish the raw IMU
  auto imu_raw_msg = std::make_unique<tobas_msgs::Imu>();
  imu_raw_msg->header.stamp = cur_time;
  driver_.getRawAccel(imu_raw_msg->accel.x(), imu_raw_msg->accel.y(), imu_raw_msg->accel.z());
  driver_.getRawGyro(imu_raw_msg->gyro.x(), imu_raw_msg->gyro.y(), imu_raw_msg->gyro.z());
  driver_.getRawDGyro(imu_raw_msg->dgyro.x(), imu_raw_msg->dgyro.y(), imu_raw_msg->dgyro.z());
  imu_raw_pub_->publish(std::move(imu_raw_msg));

  // Publish the filtered IMU
  auto imu_filt_msg = std::make_unique<tobas_msgs::Imu>();
  imu_filt_msg->header.stamp = cur_time;
  driver_.getFilteredAccel(imu_filt_msg->accel.x(), imu_filt_msg->accel.y(), imu_filt_msg->accel.z());
  driver_.getFilteredGyro(imu_filt_msg->gyro.x(), imu_filt_msg->gyro.y(), imu_filt_msg->gyro.z());
  driver_.getFilteredDGyro(imu_filt_msg->dgyro.x(), imu_filt_msg->dgyro.y(), imu_filt_msg->dgyro.z());
  imu_filt_pub_->publish(std::move(imu_filt_msg));

  // Publish the sampling time
  sampling_time_pub_.publish(cur_time);
}
}  // namespace fc2xx
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fc2xx::PwmBattImuDriverNode)
