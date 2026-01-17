#include "tobas_ssh_client/ssh_client.hpp"

#include <tobas_ros2_tools/register.hpp>

using namespace tobas_ssh_msgs::srv;
namespace fs = std::filesystem;

namespace ssh
{
SshClient::SshClient(rclcpp::Node::SharedPtr node)
  : node_(node)
  , set_endpoint_sc_(node, kSetEndpointSrv)
  , connect_sc_(node, kConnectSrv)
  , execute_sc_(node, kExecuteSrv)
  , scp_get_sc_(node, kSCPGetSrv)
  , scp_put_sc_(node, kSCPPutSrv)
  , sftp_read_sc_(node, kSFTPReadSrv)
  , sftp_write_sc_(node, kSFTPWriteSrv)
  , list_sc_(node, kListSrv)
{
}

bool SshClient::fileExists(const fs::path& file_path)
{
  std::string output;
  return execute("[ -f " + file_path.string() + " ]", output) == kNoError;
}

bool SshClient::dirExists(const fs::path& dir_path)
{
  std::string output;
  return execute("[ -d " + dir_path.string() + " ]", output) == kNoError;
}

SshClient::Error SshClient::setEndpoint(const std::string& host, const std::string& user)
{
  const auto req = std::make_shared<SetEndpoint::Request>();
  req->host = host;
  req->user = user;

  if (!set_endpoint_sc_.call(req)) {
    return error_code_ = kServiceNotReady;
  }

  return error_code_ = kNoError;
}

SshClient::Error SshClient::connect()
{
  const auto req = std::make_shared<Connect::Request>();

  if (!connect_sc_.call(req)) {
    return error_code_ = kServiceNotReady;
  }

  const auto res = connect_sc_.getResponse();
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

  if (!execute_sc_.call(req)) {
    return error_code_ = kServiceNotReady;
  }

  const auto res = execute_sc_.getResponse();
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

SshClient::Error SshClient::scpGet(const std::string& remote_path, const std::string& local_path)
{
  const auto req = std::make_shared<ScpGet::Request>();
  req->remote_path = remote_path;
  req->local_path = local_path;

  if (!scp_get_sc_.call(req)) {
    return error_code_ = kServiceNotReady;
  }

  const auto res = scp_get_sc_.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = kServerError;
  }

  return error_code_ = kNoError;
}

SshClient::Error SshClient::scpPut(
  const std::string& local_dir,
  const std::string& remote_dir,
  bool parents,
  const std::vector<std::string>& exclude_dirs,
  bool superuser)
{
  const auto req = std::make_shared<ScpPut::Request>();
  req->local_dir = local_dir;
  req->remote_dir = remote_dir;
  req->parents = parents;
  req->exclude_dirs = exclude_dirs;
  req->superuser = superuser;

  if (!scp_put_sc_.call(req)) {
    return error_code_ = kServiceNotReady;
  }

  const auto res = scp_put_sc_.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = kServerError;
  }

  return error_code_ = kNoError;
}

SshClient::Error SshClient::sftpRead(const std::string& remote_path, std::string& text, bool superuser)
{
  const auto req = std::make_shared<SftpRead::Request>();
  req->remote_path = remote_path;
  req->superuser = superuser;

  if (!sftp_read_sc_.call(req)) {
    return error_code_ = kServiceNotReady;
  }

  const auto res = sftp_read_sc_.getResponse();
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

  if (!sftp_write_sc_.call(req)) {
    return error_code_ = kServiceNotReady;
  }

  const auto res = sftp_write_sc_.getResponse();
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

  if (!list_sc_.call(req)) {
    return error_code_ = kServiceNotReady;
  }

  const auto res = list_sc_.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = kServerError;
  }

  dst = res->entries;
  return error_code_ = kNoError;
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
    case kServiceNotReady:
      return "Service server is not ready.";
    case kServerError:
      return server_error_msg_.c_str();
    default:
      return "Unknown error";
  }
}
}  // namespace ssh
