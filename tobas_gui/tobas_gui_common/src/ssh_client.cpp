// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/ssh_client.hpp"

#include <QMetaType>

#include <tobas_qt_tools/thread.hpp>

Q_DECLARE_METATYPE(tobas::ssh::SshClient::Error);

namespace tobas
{
namespace gui
{
namespace cmn
{
namespace
{
class SshConnectThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(tobas::ssh::SshClient::Error error);

public:
  explicit SshConnectThread(ssh::SshClient& impl) : impl_(impl)
  {
  }

  void run() override
  {
    const auto res = impl_.connect();
    Q_EMIT finished(res);
  }

private:
  ssh::SshClient& impl_;
};

class SshExecuteThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(tobas::ssh::SshClient::Error error);

public:
  explicit SshExecuteThread(
    ssh::SshClient& impl,
    const std::string& command,
    std::string& output,
    bool superuser,
    bool background)
    : impl_(impl), command_(command), output_(output), superuser_(superuser), background_(background)
  {
  }

  void run() override
  {
    const auto res = impl_.execute(command_, output_, superuser_, background_);
    Q_EMIT finished(res);
  }

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
  void finished(tobas::ssh::SshClient::Error error);

public:
  explicit ScpGetThread(ssh::SshClient& impl, const std::string& remote_path, const std::string& local_path)
    : impl_(impl), remote_path_(remote_path), local_path_(local_path)
  {
  }

  void run() override
  {
    const auto res = impl_.scpGet(remote_path_, local_path_);
    Q_EMIT finished(res);
  }

private:
  ssh::SshClient& impl_;

  const std::string remote_path_;
  const std::string local_path_;
};

class ScpPutThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(tobas::ssh::SshClient::Error error);

public:
  explicit ScpPutThread(
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

  void run() override
  {
    const auto res = impl_.scpPut(local_dir_, remote_dir_, parents_, exclude_dirs_, superuser_);
    Q_EMIT finished(res);
  }

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
  void finished(tobas::ssh::SshClient::Error error);

public:
  explicit SftpReadThread(ssh::SshClient& impl, const std::string& remote_path, std::string& text, bool superuser)
    : impl_(impl), remote_path_(remote_path), text_(text), superuser_(superuser)
  {
  }

  void run() override
  {
    const auto res = impl_.sftpRead(remote_path_, text_, superuser_);
    Q_EMIT finished(res);
  }

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
  void finished(tobas::ssh::SshClient::Error error);

public:
  explicit SftpWriteThread(ssh::SshClient& impl, const std::string& remote_path, const std::string& text, bool superuser)
    : impl_(impl), remote_path_(remote_path), text_(text), superuser_(superuser)
  {
  }

  void run() override
  {
    const auto res = impl_.sftpWrite(remote_path_, text_, superuser_);
    Q_EMIT finished(res);
  }

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
  void finished(tobas::ssh::SshClient::Error error);

public:
  explicit SshListThread(ssh::SshClient& impl, const std::string& pardir, std::vector<std::string>& dst)
    : impl_(impl), pardir_(pardir), dst_(dst)
  {
  }

  void run() override
  {
    const auto res = impl_.list(pardir_, dst_);
    Q_EMIT finished(res);
  }

private:
  ssh::SshClient& impl_;

  const std::string pardir_;
  std::vector<std::string>& dst_;
};
}  // namespace

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
  return std::get<0>(qt::startThreadAndWait(thread, &SshConnectThread::finished));
}

ssh::SshClient::Error
SshClientWrapper::execute(const std::string& command, std::string& output, bool superuser, bool background)
{
  SshExecuteThread thread(impl_, command, output, superuser, background);
  return std::get<0>(qt::startThreadAndWait(thread, &SshExecuteThread::finished));
}

ssh::SshClient::Error SshClientWrapper::execute(const std::string& command, bool superuser, bool background)
{
  std::string output;
  SshExecuteThread thread(impl_, command, output, superuser, background);
  return std::get<0>(qt::startThreadAndWait(thread, &SshExecuteThread::finished));
}

ssh::SshClient::Error SshClientWrapper::scpGet(const std::string& remote_path, const std::string& local_path)
{
  ScpGetThread thread(impl_, remote_path, local_path);
  return std::get<0>(qt::startThreadAndWait(thread, &ScpGetThread::finished));
}

ssh::SshClient::Error SshClientWrapper::scpPut(
  const std::string& local_dir,
  const std::string& remote_dir,
  bool parents,
  const std::vector<std::string>& exclude_dirs,
  bool superuser)
{
  ScpPutThread thread(impl_, local_dir, remote_dir, parents, exclude_dirs, superuser);
  return std::get<0>(qt::startThreadAndWait(thread, &ScpPutThread::finished));
}

ssh::SshClient::Error SshClientWrapper::sftpRead(const std::string& remote_path, std::string& text, bool superuser)
{
  SftpReadThread thread(impl_, remote_path, text, superuser);
  return std::get<0>(qt::startThreadAndWait(thread, &SftpReadThread::finished));
}

ssh::SshClient::Error
SshClientWrapper::sftpWrite(const std::string& remote_path, const std::string& text, bool superuser)
{
  SftpWriteThread thread(impl_, remote_path, text, superuser);
  return std::get<0>(qt::startThreadAndWait(thread, &SftpWriteThread::finished));
}

ssh::SshClient::Error SshClientWrapper::list(const std::string& pardir, std::vector<std::string>& dst)
{
  SshListThread thread(impl_, pardir, dst);
  return std::get<0>(qt::startThreadAndWait(thread, &SshListThread::finished));
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas

#include "ssh_client.moc"  // cppをMOCに加えるために必要
