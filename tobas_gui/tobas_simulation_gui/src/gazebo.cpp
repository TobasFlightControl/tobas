// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/gazebo.hpp"

#include <gz/msgs/double.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <QDebug>
#include <QThread>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/transport.hpp>
#include <tobas_linux/command_executor.hpp>
#include <tobas_linux/error.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_ros2_tools/node.hpp>
#include <tobas_std_tools/check.hpp>

namespace ch = std::chrono;

namespace tobas
{
namespace gui
{
namespace sim
{
namespace
{
class WaitUntilGazeboServerReadyThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success);

public:
  void run() override
  {
    Q_EMIT finished(gazebo::waitForMessage(msg_, gazebo::kGzStatsTopic));
  }

private:
  gz::msgs::WorldStatistics msg_;
};

class WaitUntilGazeboRenderingReadyThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success);

public:
  void run() override
  {
    Q_EMIT finished(gazebo::waitForMessage(msg_, gazebo::kGzRenderFpsTopic));
  }

private:
  gz::msgs::Double msg_;
};

class WaitUntilGazeboShutdownThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success);

public:
  explicit WaitUntilGazeboShutdownThread(rclcpp::Node::SharedPtr node, ch::milliseconds timeout)
    : node_(std::move(node)), timeout_(timeout)
  {
  }

  void run() override
  {
    Q_EMIT finished(ros2::waitUntilNodeGone(node_, "/ros_gz_bridge", timeout_));
  }

private:
  const rclcpp::Node::SharedPtr node_;
  const ch::milliseconds timeout_;
};
}  // namespace

bool waitUntilGazeboServerReady()
{
  WaitUntilGazeboServerReadyThread thread;
  return std::get<0>(qt::startThreadAndWait(thread, &WaitUntilGazeboServerReadyThread::finished));
}

bool waitUntilGazeboRenderingReady()
{
  WaitUntilGazeboRenderingReadyThread thread;
  return std::get<0>(qt::startThreadAndWait(thread, &WaitUntilGazeboRenderingReadyThread::finished));
}

bool waitUntilGazeboShutdown(rclcpp::Node::SharedPtr node, ch::milliseconds timeout)
{
  WaitUntilGazeboShutdownThread thread(std::move(node), timeout);
  return std::get<0>(qt::startThreadAndWait(thread, &WaitUntilGazeboShutdownThread::finished));
}

void killGazeboServer()
{
  // FIXME: The Gazebo server does not exit with only the `kill` command, so it is forcibly terminated.
  // This method may affect other processes.
  linux::CommandExecutor exec;
  qInfo() << "Killing all processes containing \"gz sim\".";
  TOBAS_CHECK(exec.execute("ps aux | grep \"gz sim\" | grep -v grep | awk '{ print \"kill -9\", $2 }' | sh"));
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas

#include "gazebo.moc"
