#pragma once

#include <QThread>

#include <tobas_ssh_client/ssh_client.hpp>

namespace gui
{
namespace hw
{
class ReadWPASupplicantThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit ReadWPASupplicantThread(rclcpp::Node::SharedPtr node);

  void run() override;

  const std::string& getText() const;

private:
  ssh::SSHClient ssh_client_;
  std::string text_;
};
}  // namespace hw
}  // namespace gui
