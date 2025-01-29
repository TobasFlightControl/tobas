#pragma once

#include <QThread>

#include <tobas_ssh_client/ssh_client.hpp>

namespace gui
{
namespace log
{
class CleanThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit CleanThread(rclcpp::Node::SharedPtr node);

  void run() override;

private:
  ssh::SSHClient ssh_client_;
};
}  // namespace log
}  // namespace gui
