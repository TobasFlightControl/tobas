// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include "tobas_version/version.hpp"

using namespace std;

int main()
{
  cout << tobas::version::kMajor << "." << tobas::version::kMinor << "." << tobas::version::kPatch << endl;
}
