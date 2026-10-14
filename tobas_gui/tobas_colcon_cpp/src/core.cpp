// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_colcon_cpp/core.hpp"

#include <tobas_linux/execute_command.hpp>

#include <unistd.h>

#include <format>
#include <iostream>

#include <tobas_ros2_tools/package.hpp>
#include <tobas_std_tools/error.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace colcon
{
std::expected<void, std::string> build(const fs::path& _pkg_path, const fs::path& _ws_path, const BuildOptions& _options)
{
  // Get the package name.
  const auto pkg_name = ros2::getPackageNameOf(_pkg_path);
  if (!pkg_name) {
    return std::unexpected("Failed to get the package name: " + pkg_name.error());
  }

  // Estimate the workspace path.
  const auto exec_path = ros2::estimateWorkspaceOf(_pkg_path);
  if (!exec_path) {
    return std::unexpected(
      "Failed to estimate the workspace path of '" + _pkg_path.string() + "': " + exec_path.error());
  }

  // Navigate to the estimated workspace.
  if (chdir(exec_path->c_str()) != 0) {
    return std::unexpected("Failed to navigate to '" + exec_path->string() + "': " + st::strError());
  }

  // Specify the log directory.
  if (setenv("COLCON_LOG_PATH", (_ws_path / "log").c_str(), 1) != 0) {
    return std::unexpected("Failed to set the colcon log directory path: " + st::strError());
  }

  // Create a build command.
  auto build_cmd = std::format(
    "colcon build "
    "--cmake-args -DCMAKE_BUILD_TYPE=Release "
    "--build-base {} "
    "--install-base {} "
    "--packages-up-to {} ",
    (_ws_path / "build").string(),
    (_ws_path / "install").string(),
    *pkg_name);

  // Add options.
  if (_options.parallel_workers == 0) {
    build_cmd += "--parallel-workers $(nproc) ";
  }
  else {
    build_cmd += std::format("--parallel-workers {} ", _options.parallel_workers);
  }
  if (_options.merge_install) {
    build_cmd += "--merge-install ";
  }
  if (_options.symlink_install) {
    build_cmd += "--symlink-install ";
  }
  if (_options.cmake_clean_cache) {
    build_cmd += "--cmake-clean-cache ";
  }

  // Build the Tobas project packages.
  std::cout << "Executing '" << build_cmd << "' on " << *exec_path << ".\n" << std::flush;
  if (const auto result = linux::executeCommand(build_cmd); !result) {
    return std::unexpected("Failed to build '" + *pkg_name + "':\n" + result.error());
  }

  return {};
}

std::expected<void, std::string> cleanWorkspace(const fs::path& _ws_path)
{
  // Navigate to the colcon workspace.
  if (chdir(_ws_path.c_str()) != 0) {
    return std::unexpected("Failed to navigate to '" + _ws_path.string() + "': " + st::strError());
  }

  // Clean the workspace.
  if (const auto result = linux::executeCommand("colcon clean workspace -y"); !result) {
    return std::unexpected("Failed to clean '" + _ws_path.string() + "':\n" + result.error());
  }

  return {};
}

}  // namespace colcon
}  // namespace tobas
