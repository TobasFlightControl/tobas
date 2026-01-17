#include "tobas_gcs/restart_thread.hpp"

namespace gui
{
namespace gcs
{
RestartThread::RestartThread(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void RestartThread::run()
{
  if (ssh_client_.execute("systemctl restart tobas_real.target", true) != ssh::SshClient::kNoError) {
    Q_EMIT finished(false, "Failed to restart the flight controller:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }

  Q_EMIT finished(true, "");
}
}  // namespace gcs
}  // namespace gui
