#include "../include/tobas_ssh_client/ssh_client.hpp"

#include <tobas_ros2_tools/register.hpp>

using namespace std;
using namespace tobas_ssh_msgs::srv;
namespace fs = filesystem;

namespace ssh
{
SSHClient::SSHClient(rclcpp::Node::SharedPtr node)
  : node_(node)
  , connect_sc_(node, kConnectSrv)
  , execute_sc_(node, kExecuteSrv)
  , scp_get_sc_(node, kSCPGetSrv)
  , scp_put_sc_(node, kSCPPutSrv)
  , sftp_read_sc_(node, kSFTPReadSrv)
  , sftp_write_sc_(node, kSFTPWriteSrv)
  , list_sc_(node, kListSrv)
{
}

bool SSHClient::fileExists(const fs::path& file_path)
{
  string output;
  return execute("[ -f " + file_path.string() + " ]", output) == E_NO_ERROR;
}

bool SSHClient::dirExists(const fs::path& dir_path)
{
  string output;
  return execute("[ -d " + dir_path.string() + " ]", output) == E_NO_ERROR;
}

SSHClient::error_t SSHClient::connect()
{
  const auto req = make_shared<Connect::Request>();

  if (!connect_sc_.call(req)) {
    return error_code_ = E_SERVICE_NOT_READY;
  }

  const auto res = connect_sc_.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = E_SERVER_ERROR;
  }

  return error_code_ = E_NO_ERROR;
}

SSHClient::error_t SSHClient::execute(const string& command, string& output, bool superuser, bool background)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "SSHClient::execute(" << command << ")");

  const auto req = make_shared<Execute::Request>();
  req->command = command;
  req->superuser = superuser;
  req->background = background;

  if (!execute_sc_.call(req)) {
    return error_code_ = E_SERVICE_NOT_READY;
  }

  const auto res = execute_sc_.getResponse();
  if (!res->success) {
    server_error_msg_ = res->error_output;
    return error_code_ = E_SERVER_ERROR;
  }

  output = res->output;
  return error_code_ = E_NO_ERROR;
}

SSHClient::error_t SSHClient::execute(const string& command, bool superuser, bool background)
{
  string output;
  return execute(command, output, superuser, background);
}

SSHClient::error_t SSHClient::scpGet(const string& remote_path, const string& local_path)
{
  const auto req = make_shared<SCPGet::Request>();
  req->remote_path = remote_path;
  req->local_path = local_path;

  if (!scp_get_sc_.call(req)) {
    return error_code_ = E_SERVICE_NOT_READY;
  }

  const auto res = scp_get_sc_.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = E_SERVER_ERROR;
  }

  return error_code_ = E_NO_ERROR;
}

SSHClient::error_t
SSHClient::scpPut(const string& local_dir, const string& remote_dir, const vector<string>& exclude_dirs, bool superuser)
{
  const auto req = make_shared<SCPPut::Request>();
  req->local_dir = local_dir;
  req->remote_dir = remote_dir;
  req->exclude_dirs = exclude_dirs;
  req->superuser = superuser;

  if (!scp_put_sc_.call(req)) {
    return error_code_ = E_SERVICE_NOT_READY;
  }

  const auto res = scp_put_sc_.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = E_SERVER_ERROR;
  }

  return error_code_ = E_NO_ERROR;
}

SSHClient::error_t SSHClient::sftpRead(const string& remote_path, string& text, bool superuser)
{
  const auto req = make_shared<SFTPRead::Request>();
  req->remote_path = remote_path;
  req->superuser = superuser;

  if (!sftp_read_sc_.call(req)) {
    return error_code_ = E_SERVICE_NOT_READY;
  }

  const auto res = sftp_read_sc_.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = E_SERVER_ERROR;
  }

  text = res->text;
  return error_code_ = E_NO_ERROR;
}

SSHClient::error_t SSHClient::sftpWrite(const string& remote_path, const string& text, bool superuser)
{
  const auto req = make_shared<SFTPWrite::Request>();
  req->remote_path = remote_path;
  req->text = text;
  req->superuser = superuser;

  if (!sftp_write_sc_.call(req)) {
    return error_code_ = E_SERVICE_NOT_READY;
  }

  const auto res = sftp_write_sc_.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = E_SERVER_ERROR;
  }

  return error_code_ = E_NO_ERROR;
}

SSHClient::error_t SSHClient::list(const string& pardir, vector<string>& dst)
{
  const auto req = make_shared<List::Request>();
  req->pardir = pardir;

  if (!list_sc_.call(req)) {
    return error_code_ = E_SERVICE_NOT_READY;
  }

  const auto res = list_sc_.getResponse();
  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = E_SERVER_ERROR;
  }

  dst = res->entries;
  return error_code_ = E_NO_ERROR;
}

SSHClient::error_t SSHClient::errorCode() const
{
  return error_code_;
}

const char* SSHClient::errorMessage() const
{
  switch (error_code_) {
    case E_NO_ERROR:
      return "";
    case E_SERVICE_NOT_READY:
      return "Service server is not ready.";
    case E_SERVER_ERROR:
      return server_error_msg_.c_str();
    default:
      return "Unknown error";
  }
}
}  // namespace ssh
