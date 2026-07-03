// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./command_executor.hpp"

namespace tobas
{
namespace linux
{
class GitHandler
{
public:
  explicit GitHandler();

  /* Return the user name. */
  std::string getUserName();

  /* Return the email address. */
  std::string getUserEmail();

private:
  CommandExecutor command_executor_;
};
}  // namespace linux
}  // namespace tobas
