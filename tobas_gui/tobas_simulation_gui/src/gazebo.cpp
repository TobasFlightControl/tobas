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
#include <tobas_qt_tools/thread.hpp>
#include <tobas_std_tools/check.hpp>

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
