// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_crypt/yescrypt.hpp>

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <password>" << std::endl;
    return EXIT_FAILURE;
  }

  const auto password = argv[1];

  tobas::crypt::Yescrypt crypt;
  const auto hash = crypt.crypt(password);
  std::cout << password << " -> " << hash << std::endl;
}
