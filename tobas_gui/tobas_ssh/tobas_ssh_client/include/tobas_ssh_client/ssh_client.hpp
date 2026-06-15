// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

#include <tobas_ros2_tools/definitions.hpp>
#include <tobas_ros2_tools/sync_action_client.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

#include <tobas_ssh_msgs/action/scp_get.hpp>
#include <tobas_ssh_msgs/srv/connect.hpp>
#include <tobas_ssh_msgs/srv/execute.hpp>
#include <tobas_ssh_msgs/srv/list.hpp>
#include <tobas_ssh_msgs/srv/scp_put.hpp>
#include <tobas_ssh_msgs/srv/set_endpoint.hpp>
#include <tobas_ssh_msgs/srv/sftp_read.hpp>
#include <tobas_ssh_msgs/srv/sftp_write.hpp>

namespace tobas
{
namespace ssh
{
/**
 * @brief プロパティサーバのクライアント．
 * @note ROSノードと同じスレッドで動作するコールバックの中で呼ぶとデッドロックする．
 */
class SshClient
{
  static constexpr char kSetEndpointSrv[] = "ssh/set_endpoint";
  static constexpr char kConnectSrv[] = "ssh/connect";
  static constexpr char kExecuteSrv[] = "ssh/execute";
  static constexpr char kScpPutSrv[] = "ssh/scp_put";
  static constexpr char kSftpReadSrv[] = "ssh/sftp_read";
  static constexpr char kSftpWriteSrv[] = "ssh/sftp_write";
  static constexpr char kListSrv[] = "ssh/list";

  static constexpr char kScpGetAction[] = "ssh/scp_get";

public:
  using SharedPtr = std::shared_ptr<SshClient>;

  enum Error
  {
    kNoError = 0,
    kServiceNotReady = -1,
    kServerError = -2,
  };

  explicit SshClient(rclcpp::Node::SharedPtr node);

  /* Getters */

  Error errorCode() const;
  const char* errorMessage() const;

  /* Setters */

  Error setEndpoint(const std::string& host, const std::string& user);

  /* SSH commands */

  Error connect();

  Error execute(const std::string& command, std::string& output, bool superuser = false, bool background = false);
  Error execute(const std::string& command, bool superuser = false, bool background = false);

  Error scpGet(
    const std::string& remote_path,
    const std::string& local_path,
    std::function<void(uint32_t, uint32_t)> callback = nullptr);

  Error scpPut(
    const std::string& local_dir,
    const std::string& remote_dir,
    bool parents,
    const std::vector<std::string>& exclude_dirs,
    bool superuser = false);

  Error sftpRead(const std::string& remote_path, std::string& text, bool superuser = false);

  Error sftpWrite(const std::string& remote_path, const std::string& text, bool superuser = false);

  Error list(const std::string& pardir, std::vector<std::string>& dst);

private:
  const rclcpp::Node::SharedPtr node_;

  ros2::SyncServiceClient<tobas_ssh_msgs::srv::SetEndpoint> set_endpoint_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::Connect> connect_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::Execute> execute_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::ScpPut> scp_put_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::SftpRead> sftp_read_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::SftpWrite> sftp_write_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::List> list_sc_;

  ros2::SyncActionClient<tobas_ssh_msgs::action::ScpGet> scp_get_ac_;

  Error error_code_ = kNoError;
  std::string server_error_msg_;
};
}  // namespace ssh
}  // namespace tobas
