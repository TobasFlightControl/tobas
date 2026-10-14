// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

#include "./ssh_client.hpp"

namespace tobas::gui::cmn
{
class RemoteProjectBuilder
{
public:
  explicit RemoteProjectBuilder(rclcpp::Node::SharedPtr _node);

  std::expected<QString, QString> build(const QString& _remote_proj_path);

private:
  cmn::SshClientWrapper ssh_client_;
};
}  // namespace tobas::gui::cmn
