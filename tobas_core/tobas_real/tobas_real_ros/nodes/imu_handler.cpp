// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/imu.hpp>
#include <tobas_constants/path.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_real_common/handler.hpp>
#include <tobas_real_common/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_ros2_tools/util.hpp>

#include <tobas_msgs_adapter/imu.hpp>
#include <tobas_real_msgs/srv/set_imu_params.hpp>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace tobas
{
namespace real
{
class ImuHandlerNode : public BaseNode
{
  using self = ImuHandlerNode;
  using super = BaseNode;
  using SetParams = tobas_real_msgs::srv::SetImuParams;

  static constexpr int kMeasureGyroBiasCount = 1000;  // [-]
  static constexpr double kGyroLpfCutoff = 30.;       // [Hz]
  static constexpr auto kMotionDetectedDeadTime = 3s;

public:
  explicit ImuHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum Stage
  {
    kMeasureGyroBias,
    kPublish,
  } stage_ = kMeasureGyroBias;

  // Config
  kdl::Vector acc_bias_;  // [m/s^2]

  // Gyro bias-related data
  kdl::Vector gyro_bias_;
  size_t gyro_bias_cnt_ = 0;
  std::array<algo::Kahan<double>, 3> gyro_sum_;
  dsp::LowPassFilter<kdl::Vector> gyro_lpf_;
  tobas_msgs::Imu::ConstSharedPtr imu_raw_in_;
  builtin_interfaces::msg::Time t_last_motion_detected_;

  ptree::PropertyTree pt_;

  ros2::PublisherPtr<tobas_msgs::Imu> imu_raw_pub_;
  ros2::PublisherPtr<tobas_msgs::Imu> imu_filt_pub_;
  ros2::SubscriberPtr<tobas_msgs::Imu> imu_raw_sub_;
  ros2::SubscriberPtr<tobas_msgs::Imu> imu_filt_sub_;

  ros2::ServiceServerPtr<SetParams> set_params_ss_;

  bool getConfig();
  void registerPubSub();

  void imuRawCb(const tobas_msgs::Imu::ConstSharedPtr& imu_raw_in);
  void imuFiltCb(const tobas_msgs::Imu::ConstSharedPtr& imu_filt_in);
  void setParamsCb(const SetParams::Request::ConstSharedPtr& req, const SetParams::Response::SharedPtr& res);
};

ImuHandlerNode::ImuHandlerNode(const rclcpp::NodeOptions& options)
  : super("real_imu_handler", nodeOptions_Default(options))
{
  gyro_lpf_.setCutoffFrequency(kGyroLpfCutoff);

  const auto cfg_dir = linux::isSuperUser() ? fs::path(kConfigDirRoot) : ros2::expandUser(kConfigDirHome);
  if (!pt_.initialize((cfg_dir / handler::imu::kConfigFileName))) {
    TOBAS_ERROR("Failed to initialize property tree. This node will not work.");
    return;
  }

  set_params_ss_ = createService<SetParams>(handler::imu::kSetParamSrv, &self::setParamsCb, this);

  if (!getConfig()) {
    TOBAS_ERROR("Failed to get configuration. This node will not work until they are set.");
    return;
  }

  registerPubSub();
}

bool ImuHandlerNode::getConfig()
{
  if (!pt_.get(ns(), handler::imu::kOffsetXKey, acc_bias_.x())) {
    TOBAS_ERROR("Failed to get \"", handler::imu::kOffsetXKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), handler::imu::kOffsetYKey, acc_bias_.y())) {
    TOBAS_ERROR("Failed to get \"", handler::imu::kOffsetXKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), handler::imu::kOffsetZKey, acc_bias_.z())) {
    TOBAS_ERROR("Failed to get \"", handler::imu::kOffsetXKey, "\".");
    return false;
  }

  return true;
}

void ImuHandlerNode::registerPubSub()
{
  imu_raw_pub_ = createPublisher<tobas_msgs::Imu>(::tobas::topic::kImuRaw);
  imu_filt_pub_ = createPublisher<tobas_msgs::Imu>(::tobas::topic::kImuFilt);
  imu_raw_sub_ = createSubscriber(real::topic::kImuRaw, &self::imuRawCb, this);
  imu_filt_sub_ = createSubscriber(real::topic::kImuFilt, &self::imuFiltCb, this);
}

void ImuHandlerNode::imuRawCb(const tobas_msgs::Imu::ConstSharedPtr& imu_raw_in)
{
  switch (stage_) {
    case kMeasureGyroBias: {
      const auto& cur_time = imu_raw_in->header.stamp;
      const auto& gyro_raw = imu_raw_in->gyro;

      if (!imu_raw_in_) {
        gyro_lpf_.setValue(gyro_raw);
        imu_raw_in_ = imu_raw_in;
        break;
      }

      // Pass gyro through LPF to reduce the effect of outliers and noise.
      const auto dt = (cur_time - imu_raw_in_->header.stamp).seconds();
      imu_raw_in_ = imu_raw_in;
      gyro_lpf_.update(gyro_raw, dt);
      const auto& gyro_filt = gyro_lpf_.getValue();

      // Retry if the angular velocity is too large, treating the vehicle as moving.
      if (gyro_filt.norm() > kStaticGyroThresh) {
        TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Motion was detected while measuring the gyro bias. Retrying...");
        gyro_bias_cnt_ = 0;
        for (auto& sum : gyro_sum_) {
          sum.reset();
        }
        t_last_motion_detected_ = cur_time;
        break;
      }

      // Do not measure for a fixed time after the last detected vehicle motion.
      if (cur_time - t_last_motion_detected_ < kMotionDetectedDeadTime) {
        break;
      }

      // Accumulate angular velocity.
      for (size_t i = 0; i < 3; ++i) {
        gyro_sum_[i].add(gyro_raw(i));
      }

      // Once enough data is accumulated, use the average angular velocity as the bias estimate.
      // Then move to the next stage.
      if (++gyro_bias_cnt_ == kMeasureGyroBiasCount) {
        for (size_t i = 0; i < 3; ++i) {
          gyro_bias_(i) = gyro_sum_[i].get() / kMeasureGyroBiasCount;
        }
        TOBAS_INFO("Finished measuring gyro bias. It is estimated to be: ", gyro_bias_);
        stage_ = kPublish;
      }

      break;
    }
    case kPublish: {
      // Create IMU message
      auto imu_raw_out = std::make_unique<tobas_msgs::Imu>();
      imu_raw_out->header = imu_raw_in->header;
      imu_raw_out->accel = imu_raw_in->accel - acc_bias_;
      imu_raw_out->gyro = imu_raw_in->gyro - gyro_bias_;
      imu_raw_out->dgyro = imu_raw_in->dgyro;

      // Publish message
      imu_raw_pub_->publish(std::move(imu_raw_out));

      break;
    }
  }
}

void ImuHandlerNode::imuFiltCb(const tobas_msgs::Imu::ConstSharedPtr& imu_filt_in)
{
  if (stage_ != kPublish) {
    return;
  }

  // Create IMU message
  auto imu_filt_out = std::make_unique<tobas_msgs::Imu>();
  imu_filt_out->header = imu_filt_in->header;
  imu_filt_out->accel = imu_filt_in->accel - acc_bias_;
  imu_filt_out->gyro = imu_filt_in->gyro - gyro_bias_;
  imu_filt_out->dgyro = imu_filt_in->dgyro;

  // Publish message
  imu_filt_pub_->publish(std::move(imu_filt_out));
}

void ImuHandlerNode::setParamsCb(const SetParams::Request::ConstSharedPtr& req, const SetParams::Response::SharedPtr& res)
{
  // Update parameters
  acc_bias_.x(req->offset_x);
  acc_bias_.y(req->offset_y);
  acc_bias_.z(req->offset_z);

  // Save parameters
  pt_.set(ns(), handler::imu::kOffsetXKey, req->offset_x);
  pt_.set(ns(), handler::imu::kOffsetYKey, req->offset_y);
  pt_.set(ns(), handler::imu::kOffsetZKey, req->offset_z);
  if (!pt_.save()) {
    res->success = false;
    res->message = "Failed to save parameters.";
    return;
  }

  if (!imu_raw_pub_ || !imu_filt_pub_) {
    registerPubSub();
  }

  res->success = true;
  res->message.clear();
}
}  // namespace real
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::real::ImuHandlerNode)
