// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/ssh_client.hpp"

#include <QDebug>
#include <QMetaType>

#include <tobas_qt_tools/string.hpp>
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
class SshWaitForLoaclServerThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool server_ready);

public:
  explicit SshWaitForLoaclServerThread(ssh::SshClient& impl) : impl_(impl)
  {
  }

  void run() override
  {
    Q_EMIT finished(impl_.waitForLocalServer());
  }

private:
  ssh::SshClient& impl_;
};

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
    const auto error = impl_.connect();
    Q_EMIT finished(error);
  }

private:
  ssh::SshClient& impl_;
};

class SshExecuteThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(tobas::ssh::SshClient::Error error, const QString& output);

public:
  explicit SshExecuteThread(ssh::SshClient& impl, const QString& command, bool superuser, bool background)
    : impl_(impl), command_(command), superuser_(superuser), background_(background)
  {
  }

  void run() override
  {
    std::string output;
    const auto error = impl_.execute(command_.toStdString(), output, superuser_, background_);
    Q_EMIT finished(error, QString::fromStdString(output));
  }

private:
  ssh::SshClient& impl_;

  const QString command_;
  const bool superuser_;
  const bool background_;
};

class ScpGetThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(tobas::ssh::SshClient::Error error);
  void feedbackReceived(uint64_t total_size, uint64_t transferred);

public:
  explicit ScpGetThread(
    ssh::SshClient& impl,
    const QString& remote_path,
    const QString& local_path,
    std::function<void(uint64_t, uint64_t)> callback)
    : impl_(impl), remote_path_(remote_path), local_path_(local_path)
  {
    if (callback) {
      connect(this, &ScpGetThread::feedbackReceived, this, callback, Qt::QueuedConnection);
    }
  }

  void run() override
  {
    const auto ros_cb = [this](uint64_t total_size, uint64_t transferred)
    { Q_EMIT feedbackReceived(total_size, transferred); };

    const auto error = impl_.scpGet(remote_path_.toStdString(), local_path_.toStdString(), ros_cb);
    Q_EMIT finished(error);
  }

private:
  ssh::SshClient& impl_;

  const QString remote_path_;
  const QString local_path_;
};

class ScpPutThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(tobas::ssh::SshClient::Error error);
  void feedbackReceived(uint64_t total_size, uint64_t transferred);

public:
  explicit ScpPutThread(
    ssh::SshClient& impl,
    const QString& local_dir,
    const QString& remote_dir,
    bool parents,
    const QStringList& exclude_dirs,
    bool superuser,
    std::function<void(uint64_t, uint64_t)> callback)
    : impl_(impl)
    , local_dir_(local_dir)
    , remote_dir_(remote_dir)
    , parents_(parents)
    , exclude_dirs_(exclude_dirs)
    , superuser_(superuser)
  {
    if (callback) {
      connect(this, &ScpPutThread::feedbackReceived, this, callback, Qt::QueuedConnection);
    }
  }

  void run() override
  {
    const auto ros_cb = [this](uint64_t total_size, uint64_t transferred)
    { Q_EMIT feedbackReceived(total_size, transferred); };

    const auto error = impl_.scpPut(
      local_dir_.toStdString(),
      remote_dir_.toStdString(),
      parents_,
      qt::stringListFromQtToStd(exclude_dirs_),
      superuser_,
      ros_cb);
    Q_EMIT finished(error);
  }

private:
  ssh::SshClient& impl_;

  const QString local_dir_;
  const QString remote_dir_;
  bool parents_;
  const QStringList exclude_dirs_;
  bool superuser_;
};

class SftpReadThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(tobas::ssh::SshClient::Error error, const QString& text);

public:
  explicit SftpReadThread(ssh::SshClient& impl, const QString& remote_path, bool superuser)
    : impl_(impl), remote_path_(remote_path), superuser_(superuser)
  {
  }

  void run() override
  {
    std::string text;
    const auto error = impl_.sftpRead(remote_path_.toStdString(), text, superuser_);
    Q_EMIT finished(error, QString::fromStdString(text));
  }

private:
  ssh::SshClient& impl_;

  const QString remote_path_;
  bool superuser_;
};

class SftpWriteThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(tobas::ssh::SshClient::Error error);

public:
  explicit SftpWriteThread(ssh::SshClient& impl, const QString& remote_path, const QString& text, bool superuser)
    : impl_(impl), remote_path_(remote_path), text_(text), superuser_(superuser)
  {
  }

  void run() override
  {
    const auto error = impl_.sftpWrite(remote_path_.toStdString(), text_.toStdString(), superuser_);
    Q_EMIT finished(error);
  }

private:
  ssh::SshClient& impl_;

  const QString remote_path_;
  const QString text_;
  bool superuser_;
};

class SshListThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(tobas::ssh::SshClient::Error error, const QStringList& list);

public:
  explicit SshListThread(ssh::SshClient& impl, const QString& pardir) : impl_(impl), pardir_(pardir)
  {
  }

  void run() override
  {
    std::vector<std::string> list;
    const auto error = impl_.list(pardir_.toStdString(), list);
    Q_EMIT finished(error, qt::stringListFromStdToQt(list));
  }

private:
  ssh::SshClient& impl_;

  const QString pardir_;
};
}  // namespace

SshClientWrapper::SshClientWrapper(rclcpp::Node::SharedPtr node) : impl_(node)
{
}

bool SshClientWrapper::waitForLocalServer()
{
  SshWaitForLoaclServerThread thread(impl_);
  return std::get<0>(qt::startThreadAndWait(thread, &SshWaitForLoaclServerThread::finished));
}

ssh::SshClient::Error SshClientWrapper::errorCode() const
{
  return impl_.errorCode();
}

const char* SshClientWrapper::errorMessage() const
{
  return impl_.errorMessage();
}

bool SshClientWrapper::setEndpoint(const QString& host, const QString& user)
{
  qInfo().noquote().nospace() << "Setting the SSH endpoint to " << endpoint(host, user) << ".";
  const auto res = impl_.setEndpoint(host.toStdString(), user.toStdString());
  host_ = host;
  user_ = user;
  return res;
}

ssh::SshClient::Error SshClientWrapper::connect()
{
  SshConnectThread thread(impl_);
  qInfo().noquote().nospace() << "Connecting to " << endpoint() << " via SSH.";
  return std::get<0>(qt::startThreadAndWait(thread, &SshConnectThread::finished));
}

ssh::SshClient::Error SshClientWrapper::execute(const QString& command, QString& output, bool superuser, bool background)
{
  SshExecuteThread thread(impl_, command, superuser, background);
  ssh::SshClient::Error error;
  qInfo().noquote().nospace() << "Executing \"" << command << "\" via SSH on " << endpoint() << ".";
  std::tie(error, output) = qt::startThreadAndWait(thread, &SshExecuteThread::finished);
  return error;
}

ssh::SshClient::Error SshClientWrapper::execute(const QString& command, bool superuser, bool background)
{
  QString output;
  return execute(command, output, superuser, background);
}

ssh::SshClient::Error SshClientWrapper::scpGet(
  const QString& remote_path,
  const QString& local_path,
  std::function<void(uint64_t, uint64_t)> callback)
{
  ScpGetThread thread(impl_, remote_path, local_path, callback);
  qInfo().noquote().nospace() << "Getting " << remote_path << " via SCP from " << endpoint() << ".";
  return std::get<0>(qt::startThreadAndWait(thread, &ScpGetThread::finished));
}

ssh::SshClient::Error SshClientWrapper::scpPut(
  const QString& local_dir,
  const QString& remote_dir,
  bool parents,
  const QStringList& exclude_dirs,
  bool superuser,
  std::function<void(uint64_t, uint64_t)> callback)
{
  ScpPutThread thread(impl_, local_dir, remote_dir, parents, exclude_dirs, superuser, callback);
  qInfo().noquote().nospace() << "Sending " << local_dir << " via SCP to " << endpoint() << ".";
  return std::get<0>(qt::startThreadAndWait(thread, &ScpPutThread::finished));
}

ssh::SshClient::Error SshClientWrapper::sftpRead(const QString& remote_path, QString& text, bool superuser)
{
  SftpReadThread thread(impl_, remote_path, superuser);
  ssh::SshClient::Error error;
  qInfo().noquote().nospace() << "Reading " << remote_path << " via SFTP from " << endpoint() << ".";
  std::tie(error, text) = qt::startThreadAndWait(thread, &SftpReadThread::finished);
  return error;
}

ssh::SshClient::Error SshClientWrapper::sftpWrite(const QString& remote_path, const QString& text, bool superuser)
{
  SftpWriteThread thread(impl_, remote_path, text, superuser);
  qInfo().noquote().nospace() << "Writing " << remote_path << " via SFTP to " << endpoint() << ".";
  return std::get<0>(qt::startThreadAndWait(thread, &SftpWriteThread::finished));
}

ssh::SshClient::Error SshClientWrapper::list(const QString& pardir, QStringList& list)
{
  SshListThread thread(impl_, pardir);
  ssh::SshClient::Error error;
  qInfo().noquote().nospace() << "Getting the contents of " << pardir << " via SSH to " << endpoint() << ".";
  std::tie(error, list) = qt::startThreadAndWait(thread, &SshListThread::finished);
  return error;
}

QString SshClientWrapper::endpoint() const
{
  return endpoint(host_, user_);
}

QString SshClientWrapper::endpoint(const QString& host, const QString& user)
{
  return user + "@" + host;
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas

#include "ssh_client.moc"  // Required to add this cpp file to MOC.
