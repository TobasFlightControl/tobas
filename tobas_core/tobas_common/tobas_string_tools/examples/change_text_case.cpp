// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string title = "Title Case Example";
  const string snake = "snake_case_example";
  const string pascal = "PascalCaseExample";

  cout << "Pascal from Title: " << title << " -> " << tobas::str::pascalFromTitle(title) << endl;
  cout << "Pascal from Snake: " << snake << " -> " << tobas::str::pascalFromSnake(snake) << endl;
  cout << "Title from Snake: " << snake << " -> " << tobas::str::titleFromSnake(snake) << endl;
  cout << "Snake from Pascal: " << pascal << " -> " << tobas::str::snakeFromPascal(pascal) << endl;
  cout << "Snake from Title: " << title << " -> " << tobas::str::snakeFromTitle(title) << endl;
}
