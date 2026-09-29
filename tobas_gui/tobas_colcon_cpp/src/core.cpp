// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_colcon_cpp/core.hpp"

#include <tobas_linux/execute_command.hpp>

#include <unistd.h>

#include <format>
#include <iostream>

#include <tobas_linux/error.hpp>
#include <tobas_ros2_tools/package.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace colcon
{
Colcon::Colcon()
{
}

std::expected<void, std::string> Colcon::build(const fs::path& pkg_path, const fs::path& ws_path)
{
  // Get the package name.
  const auto pkg_name = ros2::getPackageNameOf(pkg_path);
  if (!pkg_name) {
    return std::unexpected("Failed to get the package name: " + pkg_name.error());
  }

  // Estimate the workspace path.
  const auto exec_path = ros2::estimateWorkspaceOf(pkg_path);
  if (!exec_path) {
    return std::unexpected("Failed to estimate the workspace path of '" + pkg_path.string() + "': " + exec_path.error());
  }

  // Navigate to the estimated workspace.
  if (chdir(exec_path->c_str()) != 0) {
    return std::unexpected("Failed to navigate to '" + exec_path->string() + "': " + linux::strError());
  }

  // Specify the log directory.
  if (setenv("COLCON_LOG_PATH", logBase(ws_path).c_str(), 1) != 0) {
    return std::unexpected("Failed to set the colcon log directory path: " + linux::strError());
  }

  // Create a build command.
  auto build_cmd = std::format(
    "colcon build "
    "--cmake-args -DCMAKE_BUILD_TYPE=Release "
    "--build-base {} "
    "--install-base {} "
    "--packages-up-to {} ",
    buildBase(ws_path).string(),
    installBase(ws_path).string(),
    *pkg_name);

  // Add options.
  if (build_opts_.parallel_workers == 0) {
    build_cmd += "--parallel-workers $(nproc) ";
  }
  else {
    build_cmd += std::format("--parallel-workers {} ", build_opts_.parallel_workers);
  }
  if (build_opts_.merge_install) {
    build_cmd += "--merge-install ";
  }
  if (build_opts_.symlink_install) {
    build_cmd += "--symlink-install ";
  }
  if (build_opts_.cmake_clean_cache) {
    build_cmd += "--cmake-clean-cache ";
  }

  // Build the Tobas project packages.
  std::cout << "Executing '" << build_cmd << "' on " << *exec_path << "." << std::endl;
  if (const auto result = linux::executeCommand(build_cmd); !result) {
    return std::unexpected("Failed to build '" + *pkg_name + "':\n" + result.error());
  }

  return {};
}

std::expected<void, std::string> Colcon::cleanWorkspace(const fs::path& ws_path)
{
  // Navigate to the colcon workspace.
  if (chdir(ws_path.c_str()) != 0) {
    return std::unexpected("Failed to navigate to '" + ws_path.string() + "': " + linux::strError());
  }

  // Clean the workspace.
  if (const auto result = linux::executeCommand("colcon clean workspace -y"); !result) {
    return std::unexpected("Failed to clean '" + ws_path.string() + "':\n" + result.error());
  }

  return {};
}

void Colcon::setParallelWorkers(size_t num)
{
  build_opts_.parallel_workers = num;
}

void Colcon::setMergeInstall(bool enabled)
{
  build_opts_.merge_install = enabled;
}

void Colcon::setSymlinkInstall(bool enabled)
{
  build_opts_.symlink_install = enabled;
}

void Colcon::setCmakeCleanCache(bool enabled)
{
  build_opts_.cmake_clean_cache = enabled;
}

fs::path Colcon::buildBase(const fs::path& ws_path)
{
  return ws_path / "build";
}

fs::path Colcon::installBase(const fs::path& ws_path)
{
  return ws_path / "install";
}

fs::path Colcon::logBase(const fs::path& ws_path)
{
  return ws_path / "log";
}
}  // namespace colcon
}  // namespace tobas
