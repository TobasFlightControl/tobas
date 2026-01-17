#include "tobas_gui_common/ssh_client.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace cmn
{
SshClientWrapper::SshClientWrapper(rclcpp::Node::SharedPtr node) : impl_(node)
{
}

ssh::SshClient::Error SshClientWrapper::errorCode() const
{
  return impl_.errorCode();
}

const char* SshClientWrapper::errorMessage() const
{
  return impl_.errorMessage();
}

ssh::SshClient::Error SshClientWrapper::setEndpoint(const std::string& host, const std::string& user)
{
  return impl_.setEndpoint(host, user);
}

ssh::SshClient::Error SshClientWrapper::connect()
{
  SshConnectThread thread(impl_);
  return run(thread);
}

ssh::SshClient::Error
SshClientWrapper::execute(const std::string& command, std::string& output, bool superuser, bool background)
{
  SshExecuteThread thread(impl_, command, output, superuser, background);
  return run(thread);
}

ssh::SshClient::Error SshClientWrapper::execute(const std::string& command, bool superuser, bool background)
{
  std::string output;
  SshExecuteThread thread(impl_, command, output, superuser, background);
  return run(thread);
}

ssh::SshClient::Error SshClientWrapper::scpGet(const std::string& remote_path, const std::string& local_path)
{
  ScpGetThread thread(impl_, remote_path, local_path);
  return run(thread);
}

ssh::SshClient::Error SshClientWrapper::scpPut(
  const std::string& local_dir,
  const std::string& remote_dir,
  bool parents,
  const std::vector<std::string>& exclude_dirs,
  bool superuser)
{
  ScpPutThread thread(impl_, local_dir, remote_dir, parents, exclude_dirs, superuser);
  return run(thread);
}

ssh::SshClient::Error SshClientWrapper::sftpRead(const std::string& remote_path, std::string& text, bool superuser)
{
  SftpReadThread thread(impl_, remote_path, text, superuser);
  return run(thread);
}

ssh::SshClient::Error
SshClientWrapper::sftpWrite(const std::string& remote_path, const std::string& text, bool superuser)
{
  SftpWriteThread thread(impl_, remote_path, text, superuser);
  return run(thread);
}

ssh::SshClient::Error SshClientWrapper::list(const std::string& pardir, std::vector<std::string>& dst)
{
  SshListThread thread(impl_, pardir, dst);
  return run(thread);
}

template <typename ThreadType>
ssh::SshClient::Error SshClientWrapper::run(ThreadType& thread)
{
  // 別スレッドの結果をキャッチするためのイベントループを用意
  ssh::SshClient::Error error;
  QEventLoop loop;
  QObject::connect(
    &thread,
    &ThreadType::finished,
    [&error, &loop](ssh::SshClient::Error _error)
    {
      error = _error;
      loop.quit();
    });

  // イベントループを回しながらスレッドが終了するまで待機
  thread.start();
  loop.exec();
  thread.wait();

  return error;
}

SshConnectThread::SshConnectThread(ssh::SshClient& impl) : impl_(impl)
{
}

void SshConnectThread::run()
{
  const auto res = impl_.connect();
  Q_EMIT finished(res);
}

SshExecuteThread::SshExecuteThread(
  ssh::SshClient& impl,
  const std::string& command,
  std::string& output,
  bool superuser,
  bool background)
  : impl_(impl), command_(command), output_(output), superuser_(superuser), background_(background)
{
}

void SshExecuteThread::run()
{
  const auto res = impl_.execute(command_, output_, superuser_, background_);
  Q_EMIT finished(res);
}

ScpGetThread::ScpGetThread(ssh::SshClient& impl, const std::string& remote_path, const std::string& local_path)
  : impl_(impl), remote_path_(remote_path), local_path_(local_path)
{
}

void ScpGetThread::run()
{
  const auto res = impl_.scpGet(remote_path_, local_path_);
  Q_EMIT finished(res);
}

ScpPutThread::ScpPutThread(
  ssh::SshClient& impl,
  const std::string& local_dir,
  const std::string& remote_dir,
  bool parents,
  const std::vector<std::string>& exclude_dirs,
  bool superuser)
  : impl_(impl)
  , local_dir_(local_dir)
  , remote_dir_(remote_dir)
  , parents_(parents)
  , exclude_dirs_(exclude_dirs)
  , superuser_(superuser)
{
}

void ScpPutThread::run()
{
  const auto res = impl_.scpPut(local_dir_, remote_dir_, parents_, exclude_dirs_, superuser_);
  Q_EMIT finished(res);
}

SftpReadThread::SftpReadThread(ssh::SshClient& impl, const std::string& remote_path, std::string& text, bool superuser)
  : impl_(impl), remote_path_(remote_path), text_(text), superuser_(superuser)
{
}

void SftpReadThread::run()
{
  const auto res = impl_.sftpRead(remote_path_, text_, superuser_);
  Q_EMIT finished(res);
}

SftpWriteThread::SftpWriteThread(
  ssh::SshClient& impl,
  const std::string& remote_path,
  const std::string& text,
  bool superuser)
  : impl_(impl), remote_path_(remote_path), text_(text), superuser_(superuser)
{
}

void SftpWriteThread::run()
{
  const auto res = impl_.sftpWrite(remote_path_, text_, superuser_);
  Q_EMIT finished(res);
}

SshListThread::SshListThread(ssh::SshClient& impl, const std::string& pardir, std::vector<std::string>& dst)
  : impl_(impl), pardir_(pardir), dst_(dst)
{
}

void SshListThread::run()
{
  const auto res = impl_.list(pardir_, dst_);
  Q_EMIT finished(res);
}
}  // namespace cmn
}  // namespace gui
