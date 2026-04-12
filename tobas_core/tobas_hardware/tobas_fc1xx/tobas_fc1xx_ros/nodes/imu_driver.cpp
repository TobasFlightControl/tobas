// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_dsp/low_pass_filter_p1.hpp>
#include <tobas_hardware_common/base_sensor_node.hpp>
#include <tobas_ic_drivers/stmicro/ism330dlc.hpp>
#include <tobas_real_common/ros_interface.hpp>
#include <tobas_tools/imu_sampling_time_publisher.hpp>

#include <tobas_msgs/srv/configure_imu_filter.hpp>
#include <tobas_msgs_adapter/imu.hpp>

#include "./common.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace fc1xx
{
class ImuDriverNode : public hardware::BaseSensorNode
{
  static constexpr char kSpiDevice[] = "/dev/spidev0.0";
  static constexpr auto kSamplingPeriod = 1250us;  // 800Hz

  using self = ImuDriverNode;
  using super = hardware::BaseSensorNode;

public:
  explicit ImuDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  stm::ISM330DLC imu_;

  rclcpp::Time t_prev_;
  kdl::Vector acc_raw_, gyro_raw_, prev_gyro_raw_;
  dsp::LowPassFilterP1<kdl::Vector> acc_lpf_, gyro_lpf_, dgyro_lpf_;
  bool lpf_initialized_ = false;
  bool pub_switch_ = false;

  ros2::PublisherPtr<tobas_msgs::Imu> imu_raw_pub_;
  ros2::PublisherPtr<tobas_msgs::Imu> imu_filt_pub_;
  ImuSamplingTimePublisher sampling_time_pub_;

  ros2::ServiceServerPtr<tobas_msgs::srv::ConfigureImuFilter> config_ss_;

  ros2::TimerPtr initialize_timer_;

  bool initializeImuDriver();

  void configureImuFilterCb(
    const tobas_msgs::srv::ConfigureImuFilter::Request::ConstSharedPtr& req,
    const tobas_msgs::srv::ConfigureImuFilter::Response::SharedPtr& res);

  void initializeTimerCb();
  void mainTimerCb();
};

ImuDriverNode::ImuDriverNode(const rclcpp::NodeOptions& options)
  : super("fc1xx_imu_driver", nodeOptions_Default(options))
{
  prev_gyro_raw_.setZero();

  acc_lpf_.setValue(kdl::Vector::Zero());
  gyro_lpf_.setValue(kdl::Vector::Zero());
  dgyro_lpf_.setValue(kdl::Vector::Zero());

  initialize_timer_ = createWallTimer(kRetryInitializationInterval, &self::initializeTimerCb, this);
}

bool ImuDriverNode::initializeImuDriver()
{
  if (!imu_.initialize(kSpiDevice)) {
    TOBAS_ERROR("Failed to initialize IMU.");
    return false;
  }

  if (!imu_.setAccelOutputDataRate(stm::ISM330DLC::odr_xl_t::ODR_XL_833HZ)) {
    TOBAS_ERROR("Failed to set accelerometer output data rate.");
    return false;
  }

  if (!imu_.setGyroOutputDataRate(stm::ISM330DLC::odr_g_t::ODR_G_833HZ)) {
    TOBAS_ERROR("Failed to set gyroscope output data rate.");
    return false;
  }

  // 加速度がサチるのを防ぐためにスケールを大きめに設定．
  // 量子化誤差よりもセンサの固有ノイズの方が大きいため，分解能は問題にならない．
  if (!imu_.setAccelFullScale(stm::ISM330DLC::fs_xl_t::FS_XL_16G)) {
    TOBAS_ERROR("Failed to set accelerometer full scale.");
    return false;
  }

  if (!imu_.setGyroFullScale(stm::ISM330DLC::fs_g_t::FS_G_1000DPS)) {
    TOBAS_ERROR("Failed to set gyroscope full scale.");
    return false;
  }

  return true;
}

void ImuDriverNode::configureImuFilterCb(
  const tobas_msgs::srv::ConfigureImuFilter::Request::ConstSharedPtr& req,
  const tobas_msgs::srv::ConfigureImuFilter::Response::SharedPtr& res)
{
  if (!acc_lpf_.setCutoffFrequency(req->accel_cutoff)) {
    res->success = false;
    res->message = "Failed to set accel LPF cutoff frequency.";
    return;
  }

  if (!gyro_lpf_.setCutoffFrequency(req->gyro_cutoff)) {
    res->success = false;
    res->message = "Failed to set gyro LPF cutoff frequency.";
    return;
  }

  if (!dgyro_lpf_.setCutoffFrequency(req->dgyro_cutoff)) {
    res->success = false;
    res->message = "Failed to set D-gyro LPF cutoff frequency.";
    return;
  }

  lpf_initialized_ = true;

  res->success = true;
  res->message.clear();
}

void ImuDriverNode::initializeTimerCb()
{
  if (!initializeImuDriver()) {
    return;
  }

  imu_raw_pub_ = createPublisher<tobas_msgs::Imu>(real::topic::kImuRaw);
  imu_filt_pub_ = createPublisher<tobas_msgs::Imu>(real::topic::kImuFilt);
  sampling_time_pub_.initialize(shared_from_this(), now());

  config_ss_ =
    createService<tobas_msgs::srv::ConfigureImuFilter>(service::kConfigureImuFilter, &self::configureImuFilterCb, this);

  initialize_timer_->cancel();
  main_timer_ = createWallTimer(kSamplingPeriod, &self::mainTimerCb, this);

  t_prev_ = now();
}

void ImuDriverNode::mainTimerCb()
{
  // Get current time
  const auto cur_time = now();

  // Read IMU data
  if (!imu_.readImu(acc_raw_.x(), acc_raw_.y(), acc_raw_.z(), gyro_raw_.x(), gyro_raw_.y(), gyro_raw_.z())) {
    TOBAS_FATAL("Failed to read IMU.");
    return;
  }

  // Compute time difference
  const auto dt = (cur_time - t_prev_).seconds();  // [s]
  t_prev_ = cur_time;

  // Compute D-Gyro
  const auto dgyro_raw = (gyro_raw_ - prev_gyro_raw_) / dt;
  prev_gyro_raw_ = gyro_raw_;

  // Filter IMU data
  if (lpf_initialized_) {
    acc_lpf_.update(acc_raw_, dt);
    gyro_lpf_.update(gyro_raw_, dt);
    dgyro_lpf_.update(dgyro_raw, dt);
  }

  // 生データとフィルタ済みデータを交互に発行
  if (pub_switch_) {
    // Publish raw IMU message
    auto imu_raw = std::make_unique<tobas_msgs::Imu>();
    imu_raw->header.stamp = cur_time;
    imu_raw->accel = acc_raw_;
    imu_raw->gyro = gyro_raw_;
    imu_raw->dgyro = dgyro_raw;
    imu_raw_pub_->publish(std::move(imu_raw));
  }
  else {
    if (lpf_initialized_) {
      // Publish filtered IMU message
      auto imu_filt = std::make_unique<tobas_msgs::Imu>();
      imu_filt->header.stamp = cur_time;
      imu_filt->accel = acc_lpf_.getValue();
      imu_filt->gyro = gyro_lpf_.getValue();
      imu_filt->dgyro = dgyro_lpf_.getValue();
      imu_filt_pub_->publish(std::move(imu_filt));
    }
  }
  pub_switch_ = !pub_switch_;

  // Publish sampling time
  sampling_time_pub_.publish(cur_time);
}
}  // namespace fc1xx
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fc1xx::ImuDriverNode)
