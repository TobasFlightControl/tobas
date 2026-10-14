// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_crypt/yescrypt.hpp>

int main(int _argc, char** _argv)
{
  if (_argc != 2) {
    std::cerr << "Usage: " << _argv[0] << " <password>\n";
    return EXIT_FAILURE;
  }
  const auto password = _argv[1];

  tobas::crypt::Yescrypt crypt;
  const auto hash = crypt.crypt(password);
  if (!hash) {
    std::cerr << hash.error() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << password << " -> " << *hash << '\n';
}
