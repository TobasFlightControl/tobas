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

Q_SIGNALS:
  void finished(bool _success, const QString& _message);

public:
  explicit LargeVehicleMagCalibThread(const rqt::RosQtBridge& _bridge);
  ~LargeVehicleMagCalibThread();

  void run() override;

  void reset();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns);
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
  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& _mag_raw);
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& _gnss);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& _arming);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
