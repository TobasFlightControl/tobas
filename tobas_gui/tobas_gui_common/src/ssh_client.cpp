// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/ssh_client.hpp"

#include <QDebug>
#include <QMetaType>

#include <tobas_qt_tools/string.hpp>
#include <tobas_qt_tools/thread.hpp>

Q_DECLARE_METATYPE(tobas::gui::cmn::SshClientWrapper::Result);

namespace tobas
{
namespace gui
{
namespace cmn
{
namespace
{
SshClientWrapper::Result toQtResult(const ssh::SshClient::Result& _result)
{
  if (!_result) {
    return std::unexpected(QString::fromStdString(_result.error()));
  }
  return {};
}

class SshWaitForLoaclServerThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool _server_ready);

public:
  explicit SshWaitForLoaclServerThread(ssh::SshClient& _impl) : impl_(_impl)
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
  void finished(SshClientWrapper::Result _result);

public:
  explicit SshConnectThread(ssh::SshClient& _impl) : impl_(_impl)
  {
  }

  void run() override
  {
    const auto result = impl_.connect();
    Q_EMIT finished(toQtResult(result));
  }

private:
  ssh::SshClient& impl_;
};

class SshExecuteThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(SshClientWrapper::Result _result, const QString& _output);

public:
  explicit SshExecuteThread(ssh::SshClient& _impl, const QString& _command, bool _superuser, bool _background)
    : impl_(_impl), command_(_command), superuser_(_superuser), background_(_background)
  {
  }

  void run() override
  {
    std::string output;
    const auto result = impl_.execute(command_.toStdString(), output, superuser_, background_);
    Q_EMIT finished(toQtResult(result), QString::fromStdString(output));
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
  void finished(SshClientWrapper::Result _result);
  void feedbackReceived(uint64_t _total_size, uint64_t _transferred);

public:
  explicit ScpGetThread(
    ssh::SshClient& _impl,
    const QString& _remote_path,
    const QString& _local_path,
    std::function<void(uint64_t, uint64_t)> _callback)
    : impl_(_impl), remote_path_(_remote_path), local_path_(_local_path)
  {
    if (_callback) {
      connect(this, &ScpGetThread::feedbackReceived, this, _callback, Qt::QueuedConnection);
    }
  }

  void run() override
  {
    const auto ros_cb = [this](uint64_t _total_size, uint64_t _transferred)
    { Q_EMIT feedbackReceived(_total_size, _transferred); };

    const auto result = impl_.scpGet(remote_path_.toStdString(), local_path_.toStdString(), ros_cb);
    Q_EMIT finished(toQtResult(result));
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
  void finished(SshClientWrapper::Result _result);
  void feedbackReceived(uint64_t _total_size, uint64_t _transferred);

public:
  explicit ScpPutThread(
    ssh::SshClient& _impl,
    const QString& _local_dir,
    const QString& _remote_dir,
    bool _parents,
    const QStringList& _exclude_dirs,
    bool _superuser,
    std::function<void(uint64_t, uint64_t)> _callback)
    : impl_(_impl)
    , local_dir_(_local_dir)
    , remote_dir_(_remote_dir)
    , parents_(_parents)
    , exclude_dirs_(_exclude_dirs)
    , superuser_(_superuser)
  {
    if (_callback) {
      connect(this, &ScpPutThread::feedbackReceived, this, _callback, Qt::QueuedConnection);
    }
  }

  void run() override
  {
    const auto ros_cb = [this](uint64_t _total_size, uint64_t _transferred)
    { Q_EMIT feedbackReceived(_total_size, _transferred); };

    const auto result = impl_.scpPut(
      local_dir_.toStdString(),
      remote_dir_.toStdString(),
      parents_,
      qt::stringListFromQtToStd(exclude_dirs_),
      superuser_,
      ros_cb);
    Q_EMIT finished(toQtResult(result));
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
  void finished(SshClientWrapper::Result _result, const QString& _text);

public:
  explicit SftpReadThread(ssh::SshClient& _impl, const QString& _remote_path, bool _superuser)
    : impl_(_impl), remote_path_(_remote_path), superuser_(_superuser)
  {
  }

  void run() override
  {
    std::string text;
    const auto result = impl_.sftpRead(remote_path_.toStdString(), text, superuser_);
    Q_EMIT finished(toQtResult(result), QString::fromStdString(text));
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
  void finished(SshClientWrapper::Result _result);

public:
  explicit SftpWriteThread(ssh::SshClient& _impl, const QString& _remote_path, const QString& _text, bool _superuser)
    : impl_(_impl), remote_path_(_remote_path), text_(_text), superuser_(_superuser)
  {
  }

  void run() override
  {
    const auto result = impl_.sftpWrite(remote_path_.toStdString(), text_.toStdString(), superuser_);
    Q_EMIT finished(toQtResult(result));
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
  void finished(SshClientWrapper::Result _result, const QStringList& _list);

public:
  explicit SshListThread(ssh::SshClient& _impl, const QString& _pardir) : impl_(_impl), pardir_(_pardir)
  {
  }

  void run() override
  {
    std::vector<std::string> list;
    const auto result = impl_.list(pardir_.toStdString(), list);
    Q_EMIT finished(toQtResult(result), qt::stringListFromStdToQt(list));
  }

private:
  ssh::SshClient& impl_;

  const QString pardir_;
};
}  // namespace

SshClientWrapper::SshClientWrapper(rclcpp::Node::SharedPtr _node) : impl_(_node)
{
}

bool SshClientWrapper::waitForLocalServer()
{
  SshWaitForLoaclServerThread thread(impl_);
  return std::get<0>(qt::startThreadAndWait(thread, &SshWaitForLoaclServerThread::finished));
}

bool SshClientWrapper::setEndpoint(const QString& _host, const QString& _user)
{
  qInfo().noquote().nospace() << "Setting the SSH endpoint to " << _user << "@" << _host << ".";
  return impl_.setEndpoint(_host.toStdString(), _user.toStdString());
}

SshClientWrapper::Result SshClientWrapper::connect()
{
  SshConnectThread thread(impl_);
  qInfo().noquote().nospace() << "Connecting to " << endpoint() << " via SSH.";
  return std::get<0>(qt::startThreadAndWait(thread, &SshConnectThread::finished));
}

SshClientWrapper::Result
SshClientWrapper::execute(const QString& _command, QString& _output, bool _superuser, bool _background)
{
  SshExecuteThread thread(impl_, _command, _superuser, _background);
  SshClientWrapper::Result result;
  qInfo().noquote().nospace() << "Executing '" << _command << "' via SSH on " << endpoint() << ".";
  std::tie(result, _output) = qt::startThreadAndWait(thread, &SshExecuteThread::finished);
  return result;
}

SshClientWrapper::Result SshClientWrapper::execute(const QString& _command, bool _superuser, bool _background)
{
  QString output;
  return execute(_command, output, _superuser, _background);
}

SshClientWrapper::Result SshClientWrapper::scpGet(
  const QString& _remote_path,
  const QString& _local_path,
  std::function<void(uint64_t, uint64_t)> _callback)
{
  ScpGetThread thread(impl_, _remote_path, _local_path, _callback);
  qInfo().noquote().nospace() << "Getting " << _remote_path << " via SCP from " << endpoint() << ".";
  return std::get<0>(qt::startThreadAndWait(thread, &ScpGetThread::finished));
}

SshClientWrapper::Result SshClientWrapper::scpPut(
  const QString& _local_dir,
  const QString& _remote_dir,
  bool _parents,
  const QStringList& _exclude_dirs,
  bool _superuser,
  std::function<void(uint64_t, uint64_t)> _callback)
{
  ScpPutThread thread(impl_, _local_dir, _remote_dir, _parents, _exclude_dirs, _superuser, _callback);
  qInfo().noquote().nospace() << "Sending " << _local_dir << " via SCP to " << endpoint() << ".";
  return std::get<0>(qt::startThreadAndWait(thread, &ScpPutThread::finished));
}

SshClientWrapper::Result SshClientWrapper::sftpRead(const QString& _remote_path, QString& _text, bool _superuser)
{
  SftpReadThread thread(impl_, _remote_path, _superuser);
  SshClientWrapper::Result result;
  qInfo().noquote().nospace() << "Reading " << _remote_path << " via SFTP from " << endpoint() << ".";
  std::tie(result, _text) = qt::startThreadAndWait(thread, &SftpReadThread::finished);
  return result;
}

SshClientWrapper::Result SshClientWrapper::sftpWrite(const QString& _remote_path, const QString& _text, bool _superuser)
{
  SftpWriteThread thread(impl_, _remote_path, _text, _superuser);
  qInfo().noquote().nospace() << "Writing " << _remote_path << " via SFTP to " << endpoint() << ".";
  return std::get<0>(qt::startThreadAndWait(thread, &SftpWriteThread::finished));
}

SshClientWrapper::Result SshClientWrapper::list(const QString& _pardir, QStringList& _list)
{
  SshListThread thread(impl_, _pardir);
  SshClientWrapper::Result result;
  qInfo().noquote().nospace() << "Getting the contents of " << _pardir << " via SSH to " << endpoint() << ".";
  std::tie(result, _list) = qt::startThreadAndWait(thread, &SshListThread::finished);
  return result;
}

QString SshClientWrapper::endpoint() const
{
  return QString::fromStdString(impl_.endpoint());
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas

#include "ssh_client.moc"  // Required to add this cpp file to MOC.
