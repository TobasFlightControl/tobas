#include "tobas_gcs/shutdown_thread.hpp"

namespace gui
{
namespace gcs
{
ShutdownThread::ShutdownThread(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void ShutdownThread::run()
{
  if (ssh_client_.execute("poweroff", true, true) != ssh::SSHClient::E_NO_ERROR) {
    Q_EMIT finished(false, "Failed to shutdown the flight controller:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }

  Q_EMIT finished(true, "");
}
}  // namespace gcs
}  // namespace gui
