// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/recorder/start_thread.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_path_tools/join.hpp>

namespace tobas::gui::log
{
void RecordStartThread::run()
{
  if (!sc_) {
    Q_EMIT finished(false, "ROS interfaces have not been initialized.");
    return;
  }

  const auto req = std::make_shared<tobas_msgs::srv::BagRecordStart::Request>();
  req->name = log_name_.toStdString();

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

void RecordStartThread::initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns)
{
  sc_.emplace(_node, path::join(_ns, kRemoteIfaceNS, service::kRosbagRecordStart));
}

void RecordStartThread::clearRosInterfaces()
{
  sc_.reset();
}

void RecordStartThread::setLogName(const QString& _log_name)
{
  log_name_ = _log_name;
}
}  // namespace tobas::gui::log
