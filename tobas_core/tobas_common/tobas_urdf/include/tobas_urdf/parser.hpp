// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <urdf_world/types.h>

#include <tobas_ros2_tools/console_bridge/output_handler_text.hpp>

namespace tobas
{
namespace urdf
{
class Parser
{
public:
  explicit Parser();

  std::expected<::urdf::ModelInterfaceSharedPtr, std::string> parseFromPath(const std::string& path);
  std::expected<::urdf::ModelInterfaceSharedPtr, std::string> parseFromText(const std::string& xml);

private:
  console_bridge::OutputHandlerText oh_;
};
}  // namespace urdf
}  // namespace tobas
