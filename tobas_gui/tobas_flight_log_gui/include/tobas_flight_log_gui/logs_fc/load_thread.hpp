#pragma once

#include <QThread>

#include <tobas_ssh_client/ssh_client.hpp>

namespace gui
{
namespace log
{
class LoadThreadFC : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message, const QStringList& rosbag_names);

public:
  explicit LoadThreadFC(rclcpp::Node::SharedPtr node);

  void run() override;

private:
  ssh::SSHClient ssh_client_;
};
}  // namespace log
}  // namespace gui
