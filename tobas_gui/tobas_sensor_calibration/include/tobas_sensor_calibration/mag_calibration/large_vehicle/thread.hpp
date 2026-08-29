// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

#include <QThread>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_geographic/geography.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
class LargeVehicleMagCalibThread : public QThread
{
  Q_OBJECT

  using self = LargeVehicleMagCalibThread;
  using super = QThread;

  static constexpr size_t kDataCount = 200;
  static constexpr auto kCollectDataTimeout = std::chrono::seconds(10);

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit LargeVehicleMagCalibThread(const rqt::RosQtBridge& bridge);
  ~LargeVehicleMagCalibThread();

  void run() override;

  void reset();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);
  void clearRosInterfaces();

private:
  rclcpp::Node::SharedPtr node_;
  geo::Geography geography_;

  std::optional<ros2::SyncServiceClient<tobas_real_msgs::srv::SetMagnetometerParams>> set_params_sc_;

  tobas_msgs::MagneticField::ConstSharedPtr mag_raw_;
  tobas_msgs::Gnss::ConstSharedPtr gnss_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  bool get_data_ = false;
  size_t cnt_;
  std::array<algo::Kahan<double>, 3> mag_sum_;

private Q_SLOTS:
  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag_raw);
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
