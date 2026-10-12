// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <sys/types.h>

#include <expected>
#include <string>
#include <vector>

namespace tobas
{
namespace linux
{
/**
 * Return the child PID or an error if the argument list is empty or `fork()` fails.
 * Execution failures in the child are reported by exit status 127.
 */
std::expected<pid_t, std::string> createSubprocess(const std::vector<char*>& _argv);

/** Run a bash command in a subprocess. */
std::expected<pid_t, std::string> createSubprocess(const std::string& command);
}  // namespace linux
}  // namespace tobas
