// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_ssh_client/ssh_client.hpp>

namespace tobas
{
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
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
