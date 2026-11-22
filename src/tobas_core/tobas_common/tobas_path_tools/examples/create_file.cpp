// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdlib>
#include <iostream>
#include <string>

#include <tobas_path_tools/core.hpp>

int main()
{
  const std::string path = "/tmp/this/is/the/file/created/by/create_file";

  const auto res = tobas::path::createFilePath(path);
  if (!res) {
    std::cerr << res.error() << '\n';
    return EXIT_FAILURE;
  }

  std::cout << path << " is created successfully.\n";
  return EXIT_SUCCESS;
}
