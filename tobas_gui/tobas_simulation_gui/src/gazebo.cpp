#include "tobas_simulation_gui/gazebo.hpp"

#include <gz/msgs/double.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <QThread>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/transport.hpp>
#include <tobas_linux/command_executor.hpp>
#include <tobas_linux/error.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_ros2_tools/node.hpp>

using namespace std::chrono_literals;

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
    Q_EMIT finished(tobas::gazebo::waitForMessage(msg_, tobas::gazebo::kGzStatsTopic));
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
    Q_EMIT finished(tobas::gazebo::waitForMessage(msg_, tobas::gazebo::kGzRenderFpsTopic));
  }

private:
  gz::msgs::Double msg_;
};

class KillGazeboThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit KillGazeboThread(rclcpp::Node::SharedPtr node, pid_t pid) : node_(node), pid_(pid)
  {
  }

  void run() override
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

private:
  const rclcpp::Node::SharedPtr node_;
  const pid_t pid_;

  linux::CommandExecutor cmd_exec_;
};
}  // namespace

bool waitUntilGazeboServerReady()
{
  WaitUntilGazeboServerReadyThread thread;
  return std::get<0>(tobas::qt::startThreadAndWait(thread, &WaitUntilGazeboServerReadyThread::finished));
}

bool waitUntilGazeboRenderingReady()
{
  WaitUntilGazeboRenderingReadyThread thread;
  return std::get<0>(tobas::qt::startThreadAndWait(thread, &WaitUntilGazeboRenderingReadyThread::finished));
}

std::expected<void, QString> killGazebo(rclcpp::Node::SharedPtr node, pid_t pid)
{
  KillGazeboThread thread(node, pid);
  const auto [success, message] = tobas::qt::startThreadAndWait(thread, &KillGazeboThread::finished);
  if (success) {
    return {};
  }
  else {
    return std::unexpected(message);
  }
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas

#include "gazebo.moc"
