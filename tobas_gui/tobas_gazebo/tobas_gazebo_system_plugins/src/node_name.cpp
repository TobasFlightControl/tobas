// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/node_name.hpp"

#include <cctype>

namespace tobas
{
namespace gazebo
{
std::string sanitizeNodeName(std::string str)
{
  // Filter that allows only `[0-9A-Za-z_]`.
  const auto is_allowed = [](unsigned char c) { return std::isalnum(c) || c == '_'; };

  for (char& ch : str) {
    if (!is_allowed(static_cast<unsigned char>(ch))) {
      ch = '_';
    }
  }

  return str;
}
}  // namespace gazebo
}  // namespace tobas
