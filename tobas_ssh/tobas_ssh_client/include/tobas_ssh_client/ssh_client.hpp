#pragma once

#include <filesystem>

#include <tobas_ros2_tools/definitions.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

#include <tobas_ssh_msgs/srv/connect.hpp>
#include <tobas_ssh_msgs/srv/execute.hpp>
#include <tobas_ssh_msgs/srv/list.hpp>
#include <tobas_ssh_msgs/srv/scp_get.hpp>
#include <tobas_ssh_msgs/srv/scp_put.hpp>
#include <tobas_ssh_msgs/srv/sftp_read.hpp>
#include <tobas_ssh_msgs/srv/sftp_write.hpp>

namespace ssh
{
/**
 * @brief プロパティサーバのクライアント．
 * @note ROSノードと同じスレッドで動作するコールバックの中で呼ぶとデッドロックする．
 */
class SSHClient
{
  static constexpr char kConnectSrv[] = "ssh/connect";
  static constexpr char kExecuteSrv[] = "ssh/execute";
  static constexpr char kSCPGetSrv[] = "ssh/scp_get";
  static constexpr char kSCPPutSrv[] = "ssh/scp_put";
  static constexpr char kSFTPReadSrv[] = "ssh/sftp_read";
  static constexpr char kSFTPWriteSrv[] = "ssh/sftp_write";
  static constexpr char kListSrv[] = "ssh/list";

public:
  using SharedPtr = std::shared_ptr<SSHClient>;

  enum error_t
  {
    E_NO_ERROR = 0,
    E_SERVICE_NOT_READY = -1,
    E_SERVER_ERROR = -2,
  };

  explicit SSHClient(rclcpp::Node::SharedPtr node);

  error_t connect();

  error_t execute(const std::string& command, std::string& output, bool superuser = false, bool background = false);
  error_t execute(const std::string& command, bool superuser = false, bool background = false);

  error_t scpGet(const std::string& remote_path, const std::string& local_path);
  error_t scpPut(
    const std::string& local_dir,
    const std::string& remote_dir,
    const std::vector<std::string>& exclude_dirs,
    bool superuser = false);

  error_t sftpRead(const std::string& remote_path, std::string& text, bool superuser = false);
  error_t sftpWrite(const std::string& remote_path, const std::string& text, bool superuser = false);

  error_t list(const std::string& pardir, std::vector<std::string>& dst);

  bool fileExists(const std::filesystem::path& file_path);
  bool dirExists(const std::filesystem::path& dir_path);

  error_t errorCode() const;
  const char* errorMessage() const;

private:
  const rclcpp::Node::SharedPtr node_;

  ros2::SyncServiceClient<tobas_ssh_msgs::srv::Connect> connect_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::Execute> execute_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::ScpGet> scp_get_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::ScpPut> scp_put_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::SftpRead> sftp_read_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::SftpWrite> sftp_write_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::List> list_sc_;

  error_t error_code_ = E_NO_ERROR;
  std::string server_error_msg_;
};
}  // namespace ssh
