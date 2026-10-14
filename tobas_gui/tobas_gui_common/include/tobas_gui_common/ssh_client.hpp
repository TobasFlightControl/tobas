// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

#include <tobas_ssh_client/ssh_client.hpp>

namespace tobas::gui::cmn
{
/** SSH client wrapper that does not stop the GUI. */
class SshClientWrapper
{
  using Impl = ssh::SshClient;

public:
  using Result = std::expected<void, QString>;

  explicit SshClientWrapper(rclcpp::Node::SharedPtr _node);

  bool waitForLocalServer();

  bool setEndpoint(const QString& _host, const QString& _user);

  Result connect();
  Result execute(const QString& _command, QString& _output, bool _superuser = false, bool _background = false);
  Result execute(const QString& _command, bool _superuser = false, bool _background = false);
  Result scpGet(
    const QString& _remote_path,
    const QString& _local_path,
    std::function<void(uint64_t, uint64_t)> _callback = nullptr);
  Result scpPut(
    const QString& _local_dir,
    const QString& _remote_dir,
    bool _parents,
    const QStringList& _exclude_dirs,
    bool _superuser = false,
    std::function<void(uint64_t, uint64_t)> _callback = nullptr);
  Result sftpRead(const QString& _remote_path, QString& _text, bool _superuser = false);
  Result sftpWrite(const QString& _remote_path, const QString& _text, bool _superuser = false);
  Result list(const QString& _pardir, QStringList& _list);

private:
  Impl impl_;

  QString endpoint() const;
};
}  // namespace tobas::gui::cmn
