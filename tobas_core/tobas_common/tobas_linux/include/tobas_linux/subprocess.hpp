// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>
#include <vector>

namespace tobas
{
namespace linux
{
pid_t createSubprocess(const std::vector<char*>& _argv);

/* Run a bash command in a subprocess. */
pid_t createSubprocess(const std::string& command);
}  // namespace linux
}  // namespace tobas
