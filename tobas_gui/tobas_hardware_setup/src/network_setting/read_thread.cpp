#include "tobas_hardware_setup/network_setting/read_thread.hpp"
#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hw
{
ReadWPASupplicantThread::ReadWPASupplicantThread(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void ReadWPASupplicantThread::run()
{
  // SSH接続を確認
  if (ssh_client_.connect() != ssh::SSHClient::E_NO_ERROR) {
    Q_EMIT finished(false, "No SSH connection: " + QString(ssh_client_.errorMessage()));
    return;
  }

  // リモートファイルを開いて内容を読む
  if (ssh_client_.sftpRead(kWPASupplicantPath, text_, true) != ssh::SSHClient::E_NO_ERROR) {
    Q_EMIT finished(false, ssh_client_.errorMessage());
    return;
  }

  Q_EMIT finished(true, "");
}

const std::string& ReadWPASupplicantThread::getText() const
{
  return text_;
}
}  // namespace hw
}  // namespace gui
