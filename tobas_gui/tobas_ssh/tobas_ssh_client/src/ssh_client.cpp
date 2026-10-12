// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ssh_client/ssh_client.hpp"

#include <rclcpp/qos.hpp>

using namespace tobas_ssh_msgs::srv;
using namespace tobas_ssh_msgs::action;
namespace fs = std::filesystem;

namespace tobas
{
namespace ssh
{
namespace
{
constexpr char kServerNotReadyMsg[] = "The SSH server is not ready.";
}  // namespace

SshClient::SshClient(rclcpp::Node::SharedPtr node)
  : node_(node)
  , set_endpoint_sc_(node, "ssh/set_endpoint")
  , connect_sc_(node, "ssh/connect")
  , execute_sc_(node, "ssh/execute")
  , sftp_read_sc_(node, "ssh/sftp_read")
  , sftp_write_sc_(node, "ssh/sftp_write")
  , list_sc_(node, "ssh/list")
  , scp_get_ac_(node, "ssh/scp_get")
  , scp_put_ac_(node, "ssh/scp_put")
{
  endpoint_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "ssh/endpoint",
    rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::String::ConstSharedPtr& msg)
    {
      const std::lock_guard lock(mutex_);
      endpoint_ = msg->data;
    });
}

std::string SshClient::endpoint() const
{
  const std::lock_guard lock(mutex_);
  return endpoint_;
}

bool SshClient::waitForLocalServer()
{
  // The endpoint configuration service is always available, so use it to detect the server.
  return set_endpoint_sc_.waitForService();
}

bool SshClient::setEndpoint(const std::string& host, const std::string& user)
{
  const auto req = std::make_shared<SetEndpoint::Request>();
  req->host = host;
  req->user = user;

  const auto res = set_endpoint_sc_.sendRequestAndWait(req);
  return static_cast<bool>(res);
}

SshClient::Result SshClient::connect()
{
  const auto req = std::make_shared<Connect::Request>();

  const auto res = connect_sc_.sendRequestAndWait(req);
  if (!res) {
    return std::unexpected(kServerNotReadyMsg);
  }

  if (!res->success) {
    return std::unexpected(res->message);
  }

  return {};
}

SshClient::Result SshClient::execute(const std::string& command, std::string& output, bool superuser, bool background)
{
  const auto req = std::make_shared<Execute::Request>();
  req->command = command;
  req->superuser = superuser;
  req->background = background;

  const auto res = execute_sc_.sendRequestAndWait(req);
  if (!res) {
    return std::unexpected(kServerNotReadyMsg);
  }

  if (!res->success) {
    return std::unexpected(res->error_output);
  }

  output = res->output;
  return {};
}

SshClient::Result SshClient::execute(const std::string& command, bool superuser, bool background)
{
  std::string output;
  return execute(command, output, superuser, background);
}

SshClient::Result SshClient::scpGet(
  const std::string& remote_path,
  const std::string& local_path,
  std::function<void(uint64_t, uint64_t)> callback)
{
  ScpGet::Goal goal;
  goal.remote_path = remote_path;
  goal.local_path = local_path;

  std::optional<rclcpp_action::ClientGoalHandle<ScpGet>::WrappedResult> result;
  if (callback) {
    const auto feedback_cb =
      [callback](const rclcpp_action::ClientGoalHandle<ScpGet>::SharedPtr&, const ScpGet::Feedback::ConstSharedPtr& fb)
    { callback(fb->total_size, fb->transferred); };
    result = scp_get_ac_.sendGoalAndWait(goal, feedback_cb);
  }
  else {
    result = scp_get_ac_.sendGoalAndWait(goal);
  }
  if (!result) {
    return std::unexpected(kServerNotReadyMsg);
  }

  if (result->code != rclcpp_action::ResultCode::SUCCEEDED) {
    return std::unexpected(result->result->error_message);
  }

  return {};
}

SshClient::Result SshClient::scpPut(
  const std::string& local_dir,
  const std::string& remote_dir,
  bool parents,
  const std::vector<std::string>& exclude_dirs,
  bool superuser,
  std::function<void(uint64_t, uint64_t)> callback)
{
  ScpPut::Goal goal;
  goal.local_dir = local_dir;
  goal.remote_dir = remote_dir;
  goal.parents = parents;
  goal.exclude_dirs = exclude_dirs;
  goal.superuser = superuser;

  std::optional<rclcpp_action::ClientGoalHandle<ScpPut>::WrappedResult> result;
  if (callback) {
    const auto feedback_cb =
      [callback](const rclcpp_action::ClientGoalHandle<ScpPut>::SharedPtr&, const ScpPut::Feedback::ConstSharedPtr& fb)
    { callback(fb->total_size, fb->transferred); };
    result = scp_put_ac_.sendGoalAndWait(goal, feedback_cb);
  }
  else {
    result = scp_put_ac_.sendGoalAndWait(goal);
  }
  if (!result) {
    return std::unexpected(kServerNotReadyMsg);
  }

  if (result->code != rclcpp_action::ResultCode::SUCCEEDED) {
    return std::unexpected(result->result->error_message);
  }

  return {};
}

SshClient::Result SshClient::sftpRead(const std::string& remote_path, std::string& text, bool superuser)
{
  const auto req = std::make_shared<SftpRead::Request>();
  req->remote_path = remote_path;
  req->superuser = superuser;

  const auto res = sftp_read_sc_.sendRequestAndWait(req);
  if (!res) {
    return std::unexpected(kServerNotReadyMsg);
  }

  if (!res->success) {
    return std::unexpected(res->message);
  }

  text = res->text;
  return {};
}

SshClient::Result SshClient::sftpWrite(const std::string& remote_path, const std::string& text, bool superuser)
{
  const auto req = std::make_shared<SftpWrite::Request>();
  req->remote_path = remote_path;
  req->text = text;
  req->superuser = superuser;

  const auto res = sftp_write_sc_.sendRequestAndWait(req);
  if (!res) {
    return std::unexpected(kServerNotReadyMsg);
  }

  if (!res->success) {
    return std::unexpected(res->message);
  }

  return {};
}

SshClient::Result SshClient::list(const std::string& pardir, std::vector<std::string>& dst)
{
  const auto req = std::make_shared<List::Request>();
  req->pardir = pardir;

  const auto res = list_sc_.sendRequestAndWait(req);
  if (!res) {
    return std::unexpected(kServerNotReadyMsg);
  }

  if (!res->success) {
    return std::unexpected(res->message);
  }

  dst = res->entries;
  return {};
}
}  // namespace ssh
}  // namespace tobas
