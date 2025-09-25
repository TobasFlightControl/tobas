#pragma once

#include <QThread>
#include <rclcpp/node.hpp>

#include <tobas_linux/command_executor.hpp>

namespace gui
{
namespace sim
{
class KillGazeboThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit KillGazeboThread(rclcpp::Node::SharedPtr node, pid_t pid);

  void run() override;

private:
  const rclcpp::Node::SharedPtr node_;
  const pid_t pid_;

  linux::CommandExecutor cmd_exec_;
};
}  // namespace sim
}  // namespace gui
