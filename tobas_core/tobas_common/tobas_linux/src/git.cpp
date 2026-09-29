// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/git.hpp"

#include <tobas_linux/execute_command.hpp>

namespace tobas
{
namespace linux
{
std::expected<std::string, std::string> getGitUserName()
{
  const auto result = executeCommand("git config --global user.name");
  if (!result) {
    return std::unexpected(result.error());
  }

  return result;
}

std::expected<std::string, std::string> getGitUserEmail()
{
  const auto result = executeCommand("git config --global user.email");
  if (!result) {
    return std::unexpected(result.error());
  }

  return result;
}
}  // namespace linux
}  // namespace tobas
