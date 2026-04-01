// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_git/core.hpp>

int main()
{
  const auto name = tobas::git::getGitConfigValue("user.name");
  const auto email = tobas::git::getGitConfigValue("user.email");

  std::cout << "Git User Name: " << name << std::endl;
  std::cout << "Git Email: " << email << std::endl;
}
