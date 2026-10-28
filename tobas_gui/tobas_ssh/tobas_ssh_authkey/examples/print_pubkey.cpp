// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_ssh_authkey/export.hpp>
#include <tobas_ssh_authkey/parse.hpp>
#include <tobas_ssh_authkey/prettify.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_string_tools/stream.hpp>

int main(int _argc, char** _argv)
{
  if (_argc != 2) {
    std::cerr << "Usage: " << _argv[0] << " <Path>\n";
    return EXIT_FAILURE;
  }

  const auto path = _argv[1];

  const auto raw_text = tobas::str::readText(path);
  if (!raw_text) {
    std::cerr << raw_text.error() << '\n';
    return EXIT_FAILURE;
  }

  const auto original_line = tobas::str::trim(*raw_text);

  const auto data = tobas::ssh::ak::parseLine(original_line);
  if (!data) {
    std::cerr << data.error() << '\n';
    return EXIT_FAILURE;
  }

  const auto exported_line = tobas::ssh::ak::exportLine(*data);
  if (!exported_line) {
    std::cerr << exported_line.error() << '\n';
    return EXIT_FAILURE;
  }

  const auto prettified_line = tobas::ssh::ak::prettify(*data);
  if (!prettified_line) {
    std::cerr << prettified_line.error() << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "Original  : " << original_line << '\n';
  std::cout << "Exported  : " << *exported_line << '\n';
  std::cout << "Prettified: " << *prettified_line << '\n';
}
