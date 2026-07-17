// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ssh_client/ssh_client.hpp"

#include <tobas_ros2_tools/register.hpp>

using namespace tobas_ssh_msgs::srv;
using namespace tobas_ssh_msgs::action;
namespace fs = std::filesystem;

namespace tobas
{
namespace ssh
{
SshClient::SshClient(rclcpp::Node::SharedPtr node)
  : node_(node)
  , set_endpoint_sc_(node, kSetEndpointSrv)
  , connect_sc_(node, kConnectSrv)
  , execute_sc_(node, kExecuteSrv)
  , sftp_read_sc_(node, kSftpReadSrv)
  , sftp_write_sc_(node, kSftpWriteSrv)
  , list_sc_(node, kListSrv)
  , scp_get_ac_(node, kScpGetAction)
  , scp_put_ac_(node, kScpPutAction)
{
}

SshClient::Error SshClient::errorCode() const
{
  return error_code_;
}

const char* SshClient::errorMessage() const
{
  switch (error_code_) {
    case kNoError:
      return "";
    case kServerNotReady:
      return "The SSH server is not ready.";
    case kServerError:
      return server_error_msg_.c_str();
    default:
      return "Unknown error";
  }
}

SshClient::Error SshClient::setEndpoint(const std::string& host, const std::string& user)
{
  const auto req = std::make_shared<SetEndpoint::Request>();
  req->host = host;
  req->user = user;

  const auto res = set_endpoint_sc_.sendRequestAndWait(req);
  if (!res) {
    return error_code_ = kServerNotReady;
  }

  return error_code_ = kNoError;
}

SshClient::Error SshClient::connect()
{
  const auto req = std::make_shared<Connect::Request>();

  const auto res = connect_sc_.sendRequestAndWait(req);
  if (!res) {
    return error_code_ = kServerNotReady;
  }

  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = kServerError;
  }

  return error_code_ = kNoError;
}

SshClient::Error SshClient::execute(const std::string& command, std::string& output, bool superuser, bool background)
{
  const auto req = std::make_shared<Execute::Request>();
  req->command = command;
  req->superuser = superuser;
  req->background = background;

  const auto res = execute_sc_.sendRequestAndWait(req);
  if (!res) {
    return error_code_ = kServerNotReady;
  }

  if (!res->success) {
    server_error_msg_ = res->error_output;
    return error_code_ = kServerError;
  }

  output = res->output;
  return error_code_ = kNoError;
}

SshClient::Error SshClient::execute(const std::string& command, bool superuser, bool background)
{
  std::string output;
  return execute(command, output, superuser, background);
}

SshClient::Error SshClient::scpGet(
  const std::string& remote_path,
  const std::string& local_path,
  std::function<void(uint64_t, uint64_t)> callback)
{
  ScpGet::Goal goal;
  goal.remote_path = remote_path;
  goal.local_path = local_path;

  std::optional<rclcpp_action::ClientGoalHandle<ScpGet>::WrappedResult> result_opt;
  if (callback) {
    const auto feedback_cb =
      [callback](const rclcpp_action::ClientGoalHandle<ScpGet>::SharedPtr&, const ScpGet::Feedback::ConstSharedPtr& fb)
    { callback(fb->total_size, fb->transferred); };
    result_opt = scp_get_ac_.sendGoalAndWait(goal, feedback_cb);
  }
  else {
    result_opt = scp_get_ac_.sendGoalAndWait(goal);
  }
  if (!result_opt) {
    return error_code_ = kServerNotReady;
  }

  const auto& result = result_opt.value();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    server_error_msg_ = result.result->error_message;
    return error_code_ = kServerError;
  }

  return error_code_ = kNoError;
}

SshClient::Error SshClient::scpPut(
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

  std::optional<rclcpp_action::ClientGoalHandle<ScpPut>::WrappedResult> result_opt;
  if (callback) {
    const auto feedback_cb =
      [callback](const rclcpp_action::ClientGoalHandle<ScpPut>::SharedPtr&, const ScpPut::Feedback::ConstSharedPtr& fb)
    { callback(fb->total_size, fb->transferred); };
    result_opt = scp_put_ac_.sendGoalAndWait(goal, feedback_cb);
  }
  else {
    result_opt = scp_put_ac_.sendGoalAndWait(goal);
  }
  if (!result_opt) {
    return error_code_ = kServerNotReady;
  }

  const auto& result = result_opt.value();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    server_error_msg_ = result.result->error_message;
    return error_code_ = kServerError;
  }

  return error_code_ = kNoError;
}

SshClient::Error SshClient::sftpRead(const std::string& remote_path, std::string& text, bool superuser)
{
  const auto req = std::make_shared<SftpRead::Request>();
  req->remote_path = remote_path;
  req->superuser = superuser;

  const auto res = sftp_read_sc_.sendRequestAndWait(req);
  if (!res) {
    return error_code_ = kServerNotReady;
  }

  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = kServerError;
  }

  text = res->text;
  return error_code_ = kNoError;
}

SshClient::Error SshClient::sftpWrite(const std::string& remote_path, const std::string& text, bool superuser)
{
  const auto req = std::make_shared<SftpWrite::Request>();
  req->remote_path = remote_path;
  req->text = text;
  req->superuser = superuser;

  const auto res = sftp_write_sc_.sendRequestAndWait(req);
  if (!res) {
    return error_code_ = kServerNotReady;
  }

  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = kServerError;
  }

  return error_code_ = kNoError;
}

SshClient::Error SshClient::list(const std::string& pardir, std::vector<std::string>& dst)
{
  const auto req = std::make_shared<List::Request>();
  req->pardir = pardir;

  const auto res = list_sc_.sendRequestAndWait(req);
  if (!res) {
    return error_code_ = kServerNotReady;
  }

  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = kServerError;
  }

  dst = res->entries;
  return error_code_ = kNoError;
}
}  // namespace ssh
}  // namespace tobas
