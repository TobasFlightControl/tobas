// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/command_executor.hpp"

#include <iostream>
#include <memory>

using namespace std;

namespace tobas
{
namespace linux
{
CommandExecutor::CommandExecutor()
{
}

bool CommandExecutor::execute(string command)
{
  // Redirect standard error to standard output.
  // TODO: Support more complex redirection commands.
  const auto pos = command.find('>');
  if (pos == string::npos) {
    command += " 2>&1";  // Append to the end when there is no redirection.
  }
  else {
    command.insert(pos, " 2>&1 1");  // Insert in the middle so only standard output is written to the file.
  }

  // Execute the command.
  unique_ptr<FILE, int (*)(FILE*)> pipe(popen((command).c_str(), "r"), pclose);
  if (!pipe) {
    cerr << "popen() failed." << endl;
    return false;
  }

  // Read the output.
  output_.clear();
  while (fgets(buffer_.data(), buffer_.size(), pipe.get())) {
    output_ += buffer_.data();
  }

  // Remove trailing newline characters from the output.
  if (!output_.empty() && output_.back() == '\n') {
    output_.pop_back();
  }

  // Get the exit status.
  const auto status = pclose(pipe.release());
  return status == EXIT_SUCCESS;
}
}  // namespace linux
}  // namespace tobas
