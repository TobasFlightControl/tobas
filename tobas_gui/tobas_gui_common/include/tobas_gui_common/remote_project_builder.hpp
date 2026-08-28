// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./ssh_client.hpp"

namespace tobas
{
namespace gui
{
namespace cmn
{
class RemoteProjectBuilder
{
public:
  explicit RemoteProjectBuilder(rclcpp::Node::SharedPtr node);

  bool build(const QString& remote_proj_path);

  const QString& getOutput() const;
  QString getErrorMessage() const;

private:
  cmn::SshClientWrapper ssh_client_;

  QString output_;
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
