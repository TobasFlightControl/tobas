// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

#include <QThread>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include <tobas_real_msgs/srv/set_imu_params.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
class AccelCalibrationThread : public QThread
{
  Q_OBJECT

  using self = AccelCalibrationThread;
  using super = QThread;

  static constexpr size_t kDataCount = 200;
  static constexpr auto kCollectDataTimeout = std::chrono::seconds(30);
  static constexpr double kAccelBiasNormThresh = 1.0;  // [m/s^2] ISM330DLC has up to 85 mg (= 0.83 m/s^2) offset.

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit AccelCalibrationThread(const rqt::RosQtBridge& bridge);
  ~AccelCalibrationThread();

  void run() override;

  void reset();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);
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
  void imuCb(const tobas_msgs::Imu::ConstSharedPtr& imu_raw);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
