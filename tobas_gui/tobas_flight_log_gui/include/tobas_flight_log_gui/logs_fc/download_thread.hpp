#pragma once

#include <QThread>

#include <tobas_ssh_client/ssh_client.hpp>

namespace gui
{
namespace log
{
class DownloadThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit DownloadThread(rclcpp::Node::SharedPtr node);

  void run() override;

  const QString& getLogName() const;
  void setLogName(const QString& log_name);

private:
  ssh::SSHClient ssh_client_;
  QString log_name_;
};
}  // namespace log
}  // namespace gui
