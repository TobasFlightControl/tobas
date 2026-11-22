// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <string>

#include <tobas_string_tools/core.hpp>

int main()
{
  const std::string title = "Title Case Example";
  const std::string snake = "snake_case_example";
  const std::string pascal = "PascalCaseExample";

  std::cout << "Pascal from Title: " << title << " -> " << tobas::str::pascalFromTitle(title) << '\n';
  std::cout << "Pascal from Snake: " << snake << " -> " << tobas::str::pascalFromSnake(snake) << '\n';
  std::cout << "Title from Snake: " << snake << " -> " << tobas::str::titleFromSnake(snake) << '\n';
  std::cout << "Snake from Pascal: " << pascal << " -> " << tobas::str::snakeFromPascal(pascal) << '\n';
  std::cout << "Snake from Title: " << title << " -> " << tobas::str::snakeFromTitle(title) << '\n';
}
