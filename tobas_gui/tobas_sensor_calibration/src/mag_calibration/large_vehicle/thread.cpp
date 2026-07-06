// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/mag_calibration/large_vehicle/thread.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_geomag/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_real_common/handler.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_time_tools/util.hpp>

#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>

#include "tobas_sensor_calibration/constants.hpp"

namespace tobas
{
namespace gui
{
namespace sc
{
LargeVehicleMagCalibThread::LargeVehicleMagCalibThread(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : node_(node)
{
  connect(&bridge, &RosQtBridge::rawMagReceived, this, &self::magCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::gnssReceived, this, &self::gnssCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
}

void LargeVehicleMagCalibThread::run()
{
  // Confirm that required topics have been received.
  if (!mag_raw_) {
    Q_EMIT finished(false, "Magnetic field has not been received yet.");
    return;
  }
  if (!gnss_) {
    Q_EMIT finished(false, "GNSS has not been received yet.");
    return;
  }
  if (!arming_) {
    Q_EMIT finished(false, "Arming status has not been received yet.");
    return;
  }

  // Confirm that position information has been received.
  if (gnss_->fix_type != tobas_msgs::msg::Gnss::FIX_3D) {
    Q_EMIT finished(false, "GNSS is not fixed.");
    return;
  }

  // Calculate the geomagnetic reference value at the current position.
  const auto mag =
    geomag::elementsFromGeodetic(gnss_->latitude, gnss_->longitude, gnss_->altitude, tim::yearFraction());
  const kdl::Vector mag_ref(mag.north, -mag.east, -mag.down);  // Compass XYZ corresponds to NWU.

  // Initialize.
  cnt_ = 0;
  for (auto& sum : mag_sum_) {
    sum.reset();
  }

  // Start accumulating geomagnetic data.
  get_data_ = true;

  // Wait until enough data has accumulated.
  const auto clock = node_->get_clock();
  const auto start_time = clock->now();
  rclcpp::Rate rate(100.0, clock);
  while (rclcpp::ok()) {
    if (cnt_ >= kDataCount) {
      break;
    }
    if (arming_->data) {  // Force stop if the vehicle is armed while collecting data.
      Q_EMIT finished(false, "Accelerometer calibration was canceled because an arming command was issued.");
      get_data_ = false;
      return;
    }
    if (clock->now() - start_time > kCollectDataTimeout) {
      Q_EMIT finished(false, "Timeout before Magnetic field collection is completed.");
      get_data_ = false;
      return;
    }
    rate.sleep();
  }

  // Stop accumulating geomagnetic data.
  get_data_ = false;

  // Calculate the average.
  kdl::Vector mag_mean;
  for (size_t i = 0; i < 3; ++i) {
    mag_mean(i) = mag_sum_.at(i).get() / cnt_;
  }

  // Calculate the bias. (memo: 3-41)
  const auto hard_bias = mag_mean - mag_ref;
  const auto soft_bias = mag.total;

  // Create parameters.
  const auto req = std::make_shared<tobas_real_msgs::srv::SetMagnetometerParams::Request>();
  req->hard_bias.at(0) = hard_bias.x();
  req->hard_bias.at(1) = hard_bias.y();
  req->hard_bias.at(2) = hard_bias.z();
  req->soft_bias.at(0) = soft_bias;
  req->soft_bias.at(1) = soft_bias;
  req->soft_bias.at(2) = soft_bias;
  req->soft_bias.at(3) = 0.0;
  req->soft_bias.at(4) = 0.0;
  req->soft_bias.at(5) = 0.0;

  // Update parameters.
  ros2::SyncServiceClient<tobas_real_msgs::srv::SetMagnetometerParams> sc(
    node_, path::join(ns_, kRemoteIfaceNS, real::handler::mag::kSetParamSrv));
  if (!sc.call(req, kSetParamTimeout)) {
    Q_EMIT finished(false, "Failed to send calibration results.");
    return;
  }

  // Check the result.
  const auto res = sc.getResponse();
  if (!res->success) {
    Q_EMIT finished(false, "Calibration results are rejected: " + QString::fromStdString(res->message));
    return;
  }

  Q_EMIT finished(true, "");
}

void LargeVehicleMagCalibThread::reset()
{
  mag_raw_.reset();
  gnss_.reset();

  get_data_ = false;
  cnt_ = 0;

  for (auto& sum : mag_sum_) {
    sum.reset();
  }
}

void LargeVehicleMagCalibThread::setNamespace(const std::string& ns)
{
  ns_ = ns;
}

void LargeVehicleMagCalibThread::magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag_raw)
{
  mag_raw_ = mag_raw;

  if (!get_data_) {
    return;
  }

  ++cnt_;
  for (size_t i = 0; i < 3; ++i) {
    mag_sum_.at(i).add(mag_raw->mag(i));
  }
}

void LargeVehicleMagCalibThread::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss)
{
  gnss_ = gnss;
}

void LargeVehicleMagCalibThread::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
