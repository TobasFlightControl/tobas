#pragma once

#include <QThread>

#include <tobas_ssh_client/ssh_client.hpp>

namespace gui
{
namespace gcs
{
class RestartThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit RestartThread(rclcpp::Node::SharedPtr node);

  void run() override;

private:
  ssh::SshClient ssh_client_;
};
}  // namespace gcs
}  // namespace gui
