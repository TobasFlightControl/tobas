// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

#include <tobas_ssh_client/ssh_client.hpp>

namespace tobas
{
namespace gui
{
namespace cmn
{
/** SSH client wrapper that does not stop the GUI. */
class SshClientWrapper
{
  using Impl = ssh::SshClient;

public:
  using Result = std::expected<void, QString>;

  explicit SshClientWrapper(rclcpp::Node::SharedPtr node);

  bool waitForLocalServer();

  bool setEndpoint(const QString& host, const QString& user);

  Result connect();
  Result execute(const QString& command, QString& output, bool superuser = false, bool background = false);
  Result execute(const QString& command, bool superuser = false, bool background = false);
  Result scpGet(
    const QString& remote_path,
    const QString& local_path,
    std::function<void(uint64_t, uint64_t)> callback = nullptr);
  Result scpPut(
    const QString& local_dir,
    const QString& remote_dir,
    bool parents,
    const QStringList& exclude_dirs,
    bool superuser = false,
    std::function<void(uint64_t, uint64_t)> callback = nullptr);
  Result sftpRead(const QString& remote_path, QString& text, bool superuser = false);
  Result sftpWrite(const QString& remote_path, const QString& text, bool superuser = false);
  Result list(const QString& pardir, QStringList& list);

private:
  Impl impl_;

  QString endpoint() const;
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
