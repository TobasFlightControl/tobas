#include "tobas_simulation_gui/kill_gazebo_thread.hpp"

#include <signal.h>

#include <tobas_linux/error.hpp>
#include <tobas_ros2_tools/node.hpp>

using namespace std::chrono_literals;

namespace gui
{
namespace sim
{
KillGazeboThread::KillGazeboThread(rclcpp::Node::SharedPtr node, pid_t pid) : node_(node), pid_(pid)
{
}

void KillGazeboThread::run()
{
  if (pid_ < 0) {
    Q_EMIT finished(false, "Invalid PID: " + QString::number(pid_));
    return;
  }

  // Kill ROS process
  if (kill(pid_, SIGINT) != 0) {
    if (errno == ESRCH) {
      Q_EMIT finished(false, "PID " + QString::number(pid_) + " is not found.");
      return;
    }
    else {
      Q_EMIT finished(false, "Failed to kill PID " + QString::number(pid_) + ": " + linux::strError().c_str());
      return;
    }
  }

  // Kill Gazebo process
  // FIXME: killコマンドだけだとGazeboサーバが落ちないため無理やり落としているが，このやり方だと他のプロセスにも影響が及ぶ恐れがある．
  if (!cmd_exec_.execute("ps aux | grep \"gz sim\" | grep -v grep | awk '{ print \"kill -9\", $2 }' | sh")) {
    Q_EMIT finished(false, "Failed to kill Gazebo process: " + QString::fromStdString(cmd_exec_.getOutput()));
    return;
  }

  // Gazeboサーバが落ちるまで待機
  if (!ros2::waitUntilNodeGone(node_, "/tobas_ros_gz_bridge", 30s)) {
    Q_EMIT finished(false, "Timed out waiting for the Gazebo server to shut down.");
    return;
  }

  Q_EMIT finished(true, "");
}
}  // namespace sim
}  // namespace gui
