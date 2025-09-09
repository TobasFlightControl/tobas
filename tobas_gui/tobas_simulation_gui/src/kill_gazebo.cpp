#include "tobas_simulation_gui/kill_gazebo.hpp"

#include <signal.h>

#include <QDebug>

#include <tobas_linux/command_executor.hpp>
#include <tobas_linux/error.hpp>
#include <tobas_ros2_tools/node.hpp>

using namespace std::chrono_literals;

namespace gui
{
namespace sim
{
bool killGazeboLaunch(const pid_t& pid)
{
  if (pid < 0) {
    qWarning() << "Invalid PID: " << pid;
    return false;
  }

  // Kill ROS process
  if (kill(pid, SIGINT) != 0) {
    if (errno == ESRCH) {
      qWarning() << "PID " << pid << " is not found.";
      return false;
    }
    else {
      qWarning() << "Failed to kill PID " << pid << ": " << linux::strError().c_str();
      return false;
    }
  }

  // Kill Gazebo process
  // FIXME: killコマンドだけだとGazeboサーバが落ちないため無理やり落としているが，このやり方だと他のプロセスにも影響が及ぶ恐れがある．
  linux::CommandExecutor cmd_executor;
  if (!cmd_executor.execute("ps aux | grep \"gz sim\" | grep -v grep | awk '{ print \"kill -9\", $2 }' | sh")) {
    qWarning() << "Failed to kill Gazebo process: " << cmd_executor.getOutput().c_str();
    return false;
  }

  return true;
}

bool waitForGazeboToDisappear(const rclcpp::Node::SharedPtr& node)
{
  return ros2::waitUntilNodeGone(node, "/tobas_ros_gz_bridge", 30s);
}

KillGazeboThread::KillGazeboThread(rclcpp::Node::SharedPtr node) : node_(node)
{
}

void KillGazeboThread::run()
{
  if (pid_ < 0) {
    Q_EMIT finished(false, "PID is not set.");
    return;
  }

  if (!killGazeboLaunch(pid_)) {
    Q_EMIT finished(false, "Failed to kill Gazebo launch process.");
    return;
  }

  if (!waitForGazeboToDisappear(node_)) {
    Q_EMIT finished(false, "Timed out waiting for the Gazebo server to shut down.");
    return;
  }

  Q_EMIT finished(true, "");
}

bool KillGazeboThread::setProcessId(const pid_t& pid)
{
  if (pid < 0) {
    qWarning() << "Invalid PID: " << pid;
    return false;
  }

  pid_ = pid;
  return true;
}
}  // namespace sim
}  // namespace gui
