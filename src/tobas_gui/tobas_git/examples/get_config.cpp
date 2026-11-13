// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdlib>
#include <iostream>

#include <tobas_git/core.hpp>

int main()
{
  const auto name = tobas::git::getGitConfigValue("user.name");
  if (!name) {
    std::cerr << name.error() << '\n';
    return EXIT_FAILURE;
  }

  const auto email = tobas::git::getGitConfigValue("user.email");
  if (!email) {
    std::cerr << email.error() << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "Git User Name: " << *name << '\n';
  std::cout << "Git Email: " << *email << '\n';

  return EXIT_SUCCESS;
}
