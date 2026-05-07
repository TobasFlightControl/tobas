// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_linux/core.hpp>

using namespace std;

int main()
{
  if (tobas::linux::isSuperUser()) {
    cout << "Running with super privileges." << endl;
  }
  else {
    cout << "Running with regular privileges." << endl;
  }
}
