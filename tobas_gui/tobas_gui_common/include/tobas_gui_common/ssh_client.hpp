// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QString>

#include <tobas_ssh_client/ssh_client.hpp>

namespace tobas
{
namespace gui
{
namespace cmn
{
/* SSH client wrapper that does not stop the GUI. */
class SshClientWrapper
{
  using Impl = ssh::SshClient;

public:
  explicit SshClientWrapper(rclcpp::Node::SharedPtr node);

  bool waitForLocalServer();

  Impl::Error errorCode() const;
  const char* errorMessage() const;

  bool setEndpoint(const QString& host, const QString& user);

  Impl::Error connect();
  Impl::Error execute(const QString& command, QString& output, bool superuser = false, bool background = false);
  Impl::Error execute(const QString& command, bool superuser = false, bool background = false);
  Impl::Error scpGet(
    const QString& remote_path,
    const QString& local_path,
    std::function<void(uint64_t, uint64_t)> callback = nullptr);
  Impl::Error scpPut(
    const QString& local_dir,
    const QString& remote_dir,
    bool parents,
    const QStringList& exclude_dirs,
    bool superuser = false,
    std::function<void(uint64_t, uint64_t)> callback = nullptr);
  Impl::Error sftpRead(const QString& remote_path, QString& text, bool superuser = false);
  Impl::Error sftpWrite(const QString& remote_path, const QString& text, bool superuser = false);
  Impl::Error list(const QString& pardir, QStringList& list);

private:
  Impl impl_;
  QString host_, user_;

  QString endpoint() const;
  static QString endpoint(const QString& host, const QString& user);
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
