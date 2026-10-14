// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

#include <QThread>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include <tobas_real_msgs/srv/set_imu_params.hpp>

namespace tobas::gui::sc
{
class AccelCalibrationThread : public QThread
{
  Q_OBJECT

  using self = AccelCalibrationThread;

Q_SIGNALS:
  void finished(bool _success, const QString& _message);

public:
  explicit AccelCalibrationThread(const rqt::RosQtBridge& _bridge);
  ~AccelCalibrationThread();

  void run() override;

  void reset();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns);
  void clearRosInterfaces();

private:
  rclcpp::Node::SharedPtr node_;

  std::optional<ros2::SyncServiceClient<tobas_real_msgs::srv::SetImuParams>> set_params_sc_;

  tobas_msgs::Imu::ConstSharedPtr imu_raw_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  bool get_data_ = false;
  size_t cnt_;
  std::array<algo::Kahan<double>, 3> acc_sum_;

private Q_SLOTS:
  void imuCb(const tobas_msgs::Imu::ConstSharedPtr& _imu_raw);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& _arming);
};
}  // namespace tobas::gui::sc
