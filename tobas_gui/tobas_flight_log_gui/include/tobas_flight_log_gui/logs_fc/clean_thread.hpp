#pragma once

#include <QThread>

#include <tobas_ssh_client/ssh_client.hpp>

namespace gui
{
namespace log
{
class CleanThreadFC : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit CleanThreadFC(rclcpp::Node::SharedPtr node);

  void run() override;

private:
  ssh::SSHClient ssh_client_;
};
}  // namespace log
}  // namespace gui
