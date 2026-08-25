// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/recorder/start_thread.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_path_tools/join.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
void RecordStartThread::run()
{
  if (!sc_) {
    Q_EMIT finished(false, "ROS interfaces have not been initialized.");
    return;
  }

  const auto req = std::make_shared<tobas_msgs::srv::BagRecordStart::Request>();
  req->name = log_name_;

  const auto res = sc_->sendRequestAndWait(req);
  if (!res) {
    Q_EMIT finished(false, "Flight log recording service is unavailable.");
    return;
  }

  if (!res->success) {
    Q_EMIT finished(false, "Failed to start recording flight log: " + QString::fromStdString(res->message));
    return;
  }

  Q_EMIT finished(true, "");
}

void RecordStartThread::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  sc_ = std::make_shared<ros2::SyncServiceClient<tobas_msgs::srv::BagRecordStart>>(
    node, path::join(ns, kRemoteIfaceNS, service::kRosbagRecordStart));
}

void RecordStartThread::clearRosInterfaces()
{
  sc_.reset();
}

void RecordStartThread::setLogName(const std::string& log_name)
{
  log_name_ = log_name;
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
