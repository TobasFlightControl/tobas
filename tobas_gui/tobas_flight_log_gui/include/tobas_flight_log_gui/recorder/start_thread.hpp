// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

#include <QThread>
#include <rclcpp/node.hpp>

#include <tobas_ros2_tools/sync_service_client.hpp>

#include <tobas_msgs/srv/bag_record_start.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
class RecordStartThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  void run() override;

  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);
  void clearRosInterfaces();
  void setLogName(const std::string& log_name);

private:
  std::optional<ros2::SyncServiceClient<tobas_msgs::srv::BagRecordStart>> sc_;

  std::string log_name_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
