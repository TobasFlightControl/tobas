// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

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

  bool build(const std::filesystem::path& remote_proj_path);

  const std::string& getOutput() const;
  const char* getErrorMessage() const;

private:
  const rclcpp::Node::SharedPtr node_;
  cmn::SshClientWrapper ssh_client_;

  std::string output_;
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
