// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/recorder/stop_thread.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_path_tools/join.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
RecordStopThread::RecordStopThread(rclcpp::Node::SharedPtr node) : node_(node)
{
}

void RecordStopThread::run()
{
  const auto req = std::make_shared<tobas_msgs::srv::BagRecordStop::Request>();

  const auto res = sc_->sendRequestAndWait(req);
  if (!res) {
    Q_EMIT finished(false, "Flight log recording service is unavailable.", "");
    return;
  }

  if (!res->success) {
    Q_EMIT finished(false, "Failed to stop recording flight log: " + QString::fromStdString(res->message), "");
    return;
  }

  Q_EMIT finished(true, "", QString::fromStdString(res->path));
}

void RecordStopThread::setNamespace(const std::string& ns)
{
  sc_ = std::make_shared<ros2::SyncServiceClient<tobas_msgs::srv::BagRecordStop>>(
    node_, path::join(ns, kRemoteIfaceNS, service::kRosbagRecordStop));
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
