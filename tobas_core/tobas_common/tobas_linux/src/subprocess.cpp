// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/subprocess.hpp"

#include <unistd.h>

#include <iostream>

#include <tobas_std_tools/error.hpp>

namespace tobas
{
namespace linux
{
std::expected<pid_t, std::string> createSubprocess(const std::vector<char*>& _argv)
{
  if (_argv.empty()) {
    return std::unexpected("The size of command list is 0.");
  }

  // Clone the caller process.
  // At this point, there are two processes with exactly the same contents.
  const auto pid = fork();
  if (pid < 0) {
    return std::unexpected("Failed to clone the calling process: " + st::strError());
  }

  if (pid > 0) {
    // If the child process PID is returned, continue parent process handling.
    return pid;
  }
  else {
    // If PID is 0, this is the child process, so replace its contents with the given command.
    std::cout << "Executing: ";
    for (const auto& cmd_elem : _argv) {
      std::cout << cmd_elem << " ";
    }
    std::cout << std::endl;

    // Add the terminator to the argument list.
    auto argv = _argv;
    argv.push_back(nullptr);

    // Execute the child process (blocking).
    execvp(argv.front(), argv.data());

    // execvp returns only on failure.
    std::cerr << "Subprocess failed." << std::endl;
    _exit(127);  // Error code commonly used when an exec call fails.
  }
}

std::expected<pid_t, std::string> createSubprocess(const std::string& command)
{
  // Since const_cast does not copy, the command memory must remain valid when the process is created.
  std::vector<char*> argv;
  argv.push_back(const_cast<char*>("/bin/bash"));
  argv.push_back(const_cast<char*>("-c"));
  argv.push_back(const_cast<char*>(command.c_str()));

  return createSubprocess(argv);
}
}  // namespace linux
}  // namespace tobas
