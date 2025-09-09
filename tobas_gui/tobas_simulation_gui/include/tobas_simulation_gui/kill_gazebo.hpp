#pragma once

#include <QThread>
#include <rclcpp/node.hpp>

namespace gui
{
namespace sim
{
bool killGazeboLaunch(const pid_t& pid);
bool waitForGazeboToDisappear(const rclcpp::Node::SharedPtr& node);

class KillGazeboThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit KillGazeboThread(rclcpp::Node::SharedPtr node);

  void run() override;

  bool setProcessId(const pid_t& pid);

private:
  const rclcpp::Node::SharedPtr node_;

  pid_t pid_ = -1;
};
}  // namespace sim
}  // namespace gui
