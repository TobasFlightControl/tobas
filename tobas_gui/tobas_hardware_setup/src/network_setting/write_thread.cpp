#include "tobas_hardware_setup/network_setting/write_thread.hpp"
#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hw
{
WriteWPASupplicantThread::WriteWPASupplicantThread(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void WriteWPASupplicantThread::run()
{
  // SSH接続を確認
  if (ssh_client_.connect() != ssh::SSHClient::E_NO_ERROR) {
    Q_EMIT finished(false, "No SSH connection: " + QString(ssh_client_.errorMessage()));
    return;
  }

  // 設定を書き込む
  if (ssh_client_.sftpWrite(kWPASupplicantPath, text_, true) != ssh::SSHClient::E_NO_ERROR) {
    Q_EMIT finished(false, "SFTP-Write failed: " + QString(ssh_client_.errorMessage()));
    return;
  }

  // WiFiを再起動
  if (ssh_client_.execute("wpa_cli -i wlan0 reconfigure", true) != ssh::SSHClient::E_NO_ERROR) {
    Q_EMIT finished(false, "Failed to restart DHCPCD: " + QString(ssh_client_.errorMessage()));
    return;
  }

  Q_EMIT finished(true, "");
}

void WriteWPASupplicantThread::setText(const std::string& text)
{
  text_ = text;
}
}  // namespace hw
}  // namespace gui
