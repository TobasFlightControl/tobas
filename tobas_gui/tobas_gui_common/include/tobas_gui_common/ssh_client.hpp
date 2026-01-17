#pragma once

#include <QEventLoop>
#include <QThread>

#include <tobas_ssh_client/ssh_client.hpp>

namespace gui
{
namespace cmn
{
/* GUIを停止しないSSHクライアントラッパー． */
class SshClientWrapper
{
  using Impl = ssh::SshClient;

public:
  explicit SshClientWrapper(rclcpp::Node::SharedPtr node);

  Impl::Error errorCode() const;
  const char* errorMessage() const;

  Impl::Error setEndpoint(const std::string& host, const std::string& user);

  Impl::Error connect();

  Impl::Error execute(const std::string& command, std::string& output, bool superuser = false, bool background = false);
  Impl::Error execute(const std::string& command, bool superuser = false, bool background = false);

  Impl::Error scpGet(const std::string& remote_path, const std::string& local_path);

  Impl::Error scpPut(
    const std::string& local_dir,
    const std::string& remote_dir,
    bool parents,
    const std::vector<std::string>& exclude_dirs,
    bool superuser = false);

  Impl::Error sftpRead(const std::string& remote_path, std::string& text, bool superuser = false);

  Impl::Error sftpWrite(const std::string& remote_path, const std::string& text, bool superuser = false);

  Impl::Error list(const std::string& pardir, std::vector<std::string>& dst);

private:
  Impl impl_;

  template <typename ThreadType>
  Impl::Error run(ThreadType& thread);
};

class SshConnectThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(ssh::SshClient::Error error);

public:
  explicit SshConnectThread(ssh::SshClient& impl);

  void run() override;

private:
  ssh::SshClient& impl_;
};

class SshExecuteThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(ssh::SshClient::Error error);

public:
  explicit SshExecuteThread(
    ssh::SshClient& impl,
    const std::string& command,
    std::string& output,
    bool superuser,
    bool background);

  void run() override;

private:
  ssh::SshClient& impl_;

  const std::string command_;
  std::string& output_;
  const bool superuser_;
  const bool background_;
};

class ScpGetThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(ssh::SshClient::Error error);

public:
  explicit ScpGetThread(ssh::SshClient& impl, const std::string& remote_path, const std::string& local_path);

  void run() override;

private:
  ssh::SshClient& impl_;

  const std::string remote_path_;
  const std::string local_path_;
};

class ScpPutThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(ssh::SshClient::Error error);

public:
  explicit ScpPutThread(
    ssh::SshClient& impl,
    const std::string& local_dir,
    const std::string& remote_dir,
    bool parents,
    const std::vector<std::string>& exclude_dirs,
    bool superuser);

  void run() override;

private:
  ssh::SshClient& impl_;

  const std::string local_dir_;
  const std::string remote_dir_;
  bool parents_;
  const std::vector<std::string> exclude_dirs_;
  bool superuser_;
};

class SftpReadThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(ssh::SshClient::Error error);

public:
  explicit SftpReadThread(ssh::SshClient& impl, const std::string& remote_path, std::string& text, bool superuser);

  void run() override;

private:
  ssh::SshClient& impl_;

  const std::string remote_path_;
  std::string& text_;
  bool superuser_;
};

class SftpWriteThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(ssh::SshClient::Error error);

public:
  explicit SftpWriteThread(ssh::SshClient& impl, const std::string& remote_path, const std::string& text, bool superuser);

  void run() override;

private:
  ssh::SshClient& impl_;

  const std::string remote_path_;
  const std::string text_;
  bool superuser_;
};

class SshListThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(ssh::SshClient::Error error);

public:
  explicit SshListThread(ssh::SshClient& impl, const std::string& pardir, std::vector<std::string>& dst);

  void run() override;

private:
  ssh::SshClient& impl_;

  const std::string pardir_;
  std::vector<std::string>& dst_;
};
}  // namespace cmn
}  // namespace gui
