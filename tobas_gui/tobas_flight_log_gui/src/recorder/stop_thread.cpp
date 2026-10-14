// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/recorder/stop_thread.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_path_tools/join.hpp>

namespace tobas::gui::log
{
void RecordStopThread::run()
{
  if (!sc_) {
    Q_EMIT finished(false, "ROS interfaces have not been initialized.", "");
    return;
  }

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

void RecordStopThread::initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns)
{
  sc_.emplace(_node, path::join(_ns, kRemoteIfaceNS, service::kRosbagRecordStop));
}

void RecordStopThread::clearRosInterfaces()
{
  sc_.reset();
}
}  // namespace tobas::gui::log
