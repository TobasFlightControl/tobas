// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdlib>
#include <iostream>

#include <tobas_git/core.hpp>

int main()
{
  const auto name = tobas::git::getGitConfigValue("user.name");
  if (!name) {
    std::cerr << name.error() << std::endl;
    return EXIT_FAILURE;
  }

  const auto email = tobas::git::getGitConfigValue("user.email");
  if (!email) {
    std::cerr << email.error() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Git User Name: " << *name << std::endl;
  std::cout << "Git Email: " << *email << std::endl;

  return EXIT_SUCCESS;
}
