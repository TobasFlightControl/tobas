// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/git.hpp"

#include <iostream>

namespace tobas
{
namespace linux
{
GitHandler::GitHandler()
{
}

std::string GitHandler::getUserName()
{
  if (!command_executor_.execute("git config --global user.name")) {
    std::cerr << "Failed to get Git user name." << std::endl;
    return "";
  }

  return command_executor_.getOutput();
}

std::string GitHandler::getUserEmail()
{
  if (!command_executor_.execute("git config --global user.email")) {
    std::cerr << "Failed to get Git user email." << std::endl;
    return "";
  }

  return command_executor_.getOutput();
}
}  // namespace linux
}  // namespace tobas
