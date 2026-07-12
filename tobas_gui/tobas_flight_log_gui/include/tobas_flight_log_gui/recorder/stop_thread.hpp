// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QThread>
#include <rclcpp/node.hpp>

#include <tobas_ros2_tools/sync_service_client.hpp>

#include <tobas_msgs/srv/bag_record_stop.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
class RecordStopThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message, const QString& log_path);

public:
  explicit RecordStopThread(rclcpp::Node::SharedPtr node);

  void run() override;

  void setNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;
  ros2::SyncServiceClient<tobas_msgs::srv::BagRecordStop>::SharedPtr sc_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
