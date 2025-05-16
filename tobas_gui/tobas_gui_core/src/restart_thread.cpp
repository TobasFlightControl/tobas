#include "tobas_gui_core/restart_thread.hpp"

namespace gui
{
namespace core
{
RestartThread::RestartThread(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void RestartThread::run()
{
  if (ssh_client_.execute("systemctl restart tobas_real.target", true) != ssh::SSHClient::E_NO_ERROR) {
    Q_EMIT finished(false, "Failed to restart the flight controller:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }

  Q_EMIT finished(true, "");
}
}  // namespace core
}  // namespace gui
