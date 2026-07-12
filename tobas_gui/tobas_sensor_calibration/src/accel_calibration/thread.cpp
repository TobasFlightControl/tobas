// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/accel_calibration/thread.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_real_common/handler.hpp>
#include <tobas_std_tools/universal_constants.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
AccelCalibrationThread::AccelCalibrationThread(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge) : node_(node)
{
  connect(&bridge, &RosQtBridge::rawImuReceived, this, &self::imuCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
}

void AccelCalibrationThread::run()
{
  // Confirm that required topics have been received.
  if (!imu_raw_) {
    Q_EMIT finished(false, "IMU data has not been received yet.");
    return;
  }
  if (!arming_) {
    Q_EMIT finished(false, "Arming status has not been received yet.");
    return;
  }

  // Initialize.
  cnt_ = 0;
  for (auto& sum : acc_sum_) {
    sum.reset();
  }

  // Start accumulating acceleration data.
  get_data_ = true;

  // Wait until enough data has accumulated.
  const auto clock = node_->get_clock();
  const auto start_time = clock->now();
  rclcpp::Rate rate(100.0, clock);
  while (rclcpp::ok()) {
    if (cnt_ >= kDataCount) {
      break;
    }
    if (arming_->data) {  // Abort if the vehicle is armed while collecting data.
      Q_EMIT finished(false, "Accelerometer calibration was canceled because an arming command was issued.");
      get_data_ = false;
      return;
    }
    if (clock->now() - start_time > kCollectDataTimeout) {
      Q_EMIT finished(false, "Timeout before IMU data collection is completed.");
      get_data_ = false;
      return;
    }
    rate.sleep();
  }

  // Stop accumulating acceleration data.
  get_data_ = false;

  // Calculate the average.
  kdl::Vector acc_mean;
  for (size_t i = 0; i < 3; ++i) {
    acc_mean(i) = acc_sum_.at(i).get() / cnt_;
  }

  // Calculate the bias.
  const kdl::Vector acc_ref(0, 0, st::kGravity);
  const auto acc_bias = acc_mean - acc_ref;

  // Fail if the bias is abnormally large.
  if (acc_bias.norm() > kAccelBiasNormThresh) {
    Q_EMIT finished(false, "Acceleration error is too high. Verify that the FMU is correctly oriented.");
    return;
  }

  // Create parameters.
  const auto req = std::make_shared<tobas_real_msgs::srv::SetImuParams::Request>();
  req->offset_x = acc_bias.x();
  req->offset_y = acc_bias.y();
  req->offset_z = acc_bias.z();

  // Update parameters.
  const auto res = sc_->sendRequestAndWait(req);
  if (!res) {
    Q_EMIT finished(false, "Failed to send calibration results.");
    return;
  }

  // Check the result.
  if (!res->success) {
    Q_EMIT finished(false, "Calibration results are rejected: " + QString::fromStdString(res->message));
    return;
  }

  Q_EMIT finished(true, "Accelerometer calibration finished successfully.");
}

void AccelCalibrationThread::reset()
{
  imu_raw_.reset();

  get_data_ = false;
  cnt_ = 0;

  for (auto& sum : acc_sum_) {
    sum.reset();
  }
}

void AccelCalibrationThread::setNamespace(const std::string& ns)
{
  sc_ = std::make_shared<ros2::SyncServiceClient<tobas_real_msgs::srv::SetImuParams>>(
    node_, path::join(ns, kRemoteIfaceNS, real::handler::imu::kSetParamSrv));
}

void AccelCalibrationThread::imuCb(const tobas_msgs::Imu::ConstSharedPtr& imu_raw)
{
  imu_raw_ = imu_raw;

  if (!get_data_) {
    return;
  }

  ++cnt_;
  for (size_t i = 0; i < 3; ++i) {
    acc_sum_.at(i).add(imu_raw->accel(i));
  }
}

void AccelCalibrationThread::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
