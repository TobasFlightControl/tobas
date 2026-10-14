// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <mutex>
#include <string>

#include <tobas_ros2_tools/definitions.hpp>
#include <tobas_ros2_tools/sync_action_client.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

#include <std_msgs/msg/string.hpp>

#include <tobas_ssh_msgs/action/scp_get.hpp>
#include <tobas_ssh_msgs/action/scp_put.hpp>
#include <tobas_ssh_msgs/srv/connect.hpp>
#include <tobas_ssh_msgs/srv/execute.hpp>
#include <tobas_ssh_msgs/srv/list.hpp>
#include <tobas_ssh_msgs/srv/set_endpoint.hpp>
#include <tobas_ssh_msgs/srv/sftp_read.hpp>
#include <tobas_ssh_msgs/srv/sftp_write.hpp>

namespace tobas::ssh
{
/**
 * Client for the SSH server.
 *
 * @note Calling this from a callback running on the same thread as the ROS node causes a deadlock.
 */
class SshClient
{
public:
  using Result = std::expected<void, std::string>;

  explicit SshClient(rclcpp::Node::SharedPtr _node);

  /** Wait for the local server that communicates directly with the SSH server. */
  bool waitForLocalServer();

  /** Latest endpoint published by the server, or empty before the first message arrives. */
  std::string endpoint() const;

  bool setEndpoint(const std::string& _host, const std::string& _user);

  Result connect();

  Result execute(const std::string& _command, std::string& _output, bool _superuser = false, bool _background = false);
  Result execute(const std::string& _command, bool _superuser = false, bool _background = false);

  Result scpGet(
    const std::string& _remote_path,
    const std::string& _local_path,
    std::function<void(uint64_t, uint64_t)> _callback = nullptr);

  Result scpPut(
    const std::string& _local_dir,
    const std::string& _remote_dir,
    bool _parents,
    const std::vector<std::string>& _exclude_dirs,
    bool _superuser = false,
    std::function<void(uint64_t, uint64_t)> _callback = nullptr);

  Result sftpRead(const std::string& _remote_path, std::string& _text, bool _superuser = false);

  Result sftpWrite(const std::string& _remote_path, const std::string& _text, bool _superuser = false);

  Result list(const std::string& _pardir, std::vector<std::string>& _dst);

private:
  const rclcpp::Node::SharedPtr node_;

  mutable std::mutex mutex_;
  std::string endpoint_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr endpoint_sub_;

  ros2::SyncServiceClient<tobas_ssh_msgs::srv::SetEndpoint> set_endpoint_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::Connect> connect_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::Execute> execute_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::SftpRead> sftp_read_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::SftpWrite> sftp_write_sc_;
  ros2::SyncServiceClient<tobas_ssh_msgs::srv::List> list_sc_;
  ros2::SyncActionClient<tobas_ssh_msgs::action::ScpGet> scp_get_ac_;
  ros2::SyncActionClient<tobas_ssh_msgs::action::ScpPut> scp_put_ac_;
};
}  // namespace tobas::ssh
