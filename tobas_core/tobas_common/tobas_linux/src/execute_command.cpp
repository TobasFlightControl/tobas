// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/execute_command.hpp"

#include <sys/wait.h>

#include <array>
#include <cstdio>
#include <memory>

#include <tobas_std_tools/error.hpp>

namespace tobas
{
namespace linux
{
std::expected<std::string, std::string> executeCommand(std::string command)
{
  // Capture stderr together with stdout so command failures retain their diagnostic output.
  // With a simple stdout redirection, duplicate the pipe's stdout onto stderr before redirecting stdout to the file.
  // This keeps diagnostics in the pipe without mixing them into the file.
  // TODO: Support more complex shell redirection syntax.
  const auto pos = command.find('>');
  if (pos == std::string::npos) {
    command += " 2>&1";  // Append to the end when there is no redirection.
  }
  else {
    command.insert(pos, " 2>&1 1");  // Insert in the middle so only standard output is written to the file.
  }

  // popen runs the command through a shell and exposes its stdout as a readable pipe.
  // The deleter closes the pipe and waits for the child even when reading exits early.
  std::unique_ptr<FILE, int (*)(FILE*)> pipe(popen(command.c_str(), "r"), pclose);
  if (!pipe) {
    return std::unexpected("popen() failed: " + st::strError());
  }

  // Read until EOF rather than assuming the output fits in one buffer.
  // All state is local, so separate calls cannot overwrite each other's results.
  std::array<char, 128> buffer;
  std::string output;
  while (fgets(buffer.data(), buffer.size(), pipe.get())) {
    output += buffer.data();
  }
  if (ferror(pipe.get())) {
    return std::unexpected("Failed to read command output: " + st::strError());
  }

  // Strip one final newline for line-oriented commands while preserving internal newlines.
  if (!output.empty() && output.back() == '\n') {
    output.pop_back();
  }

  // pclose returns an encoded wait status, not the command's exit code.
  // Release ownership first so the RAII deleter does not close the same pipe a second time.
  const auto status = pclose(pipe.release());
  if (status == -1) {
    return std::unexpected("pclose() failed: " + st::strError());
  }
  if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
    return output;
  }

  // Distinguish a nonzero exit code from signal termination and include captured diagnostics.
  // Keep the status message even when the command produced no output.
  std::string error;
  if (WIFEXITED(status)) {
    error = "Command exited with status " + std::to_string(WEXITSTATUS(status)) + ".";
  }
  else if (WIFSIGNALED(status)) {
    error = "Command terminated by signal " + std::to_string(WTERMSIG(status)) + ".";
  }
  else {
    error = "Command failed with process status " + std::to_string(status) + ".";
  }
  if (!output.empty()) {
    error += "\n" + output;
  }
  return std::unexpected(error);
}
}  // namespace linux
}  // namespace tobas
