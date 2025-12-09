#include "tobas_colcon_cpp/core.hpp"

#include <unistd.h>

#include <format>
#include <iostream>

#include <tobas_linux/error.hpp>
#include <tobas_ros2_tools/package.hpp>

namespace fs = std::filesystem;

namespace colcon
{
Colcon::Colcon()
{
}

bool Colcon::build(const fs::path& pkg_path, const fs::path& ws_path)
{
  // Get the package name
  const auto pkg_name = ros2::getPackageNameOf(pkg_path);
  if (!pkg_name) {
    error_msg_ = "Failed to get the package name: " + pkg_name.error();
    return false;
  }

  // Estimate the workspace path
  const auto exec_path = ros2::estimateWorkspaceOf(pkg_path);
  if (!exec_path) {
    error_msg_ = "Failed to estimate the workspace path of \"" + pkg_path.string() + "\": " + exec_path.error();
    return false;
  }

  // Navigate to the estimated workspace
  if (chdir(exec_path.value().c_str()) != 0) {
    error_msg_ = "Failed to navigate to \"" + exec_path.value().string() + "\": " + linux::strError();
    return false;
  }

  // Specify the log directory
  if (setenv("COLCON_LOG_PATH", logBase(ws_path).c_str(), 1) != 0) {
    error_msg_ = "Failed to set the colcon log directory path: " + linux::strError();
    return false;
  }

  // Create a build command
  auto build_cmd = std::format(
    "colcon build "
    "--cmake-args -DCMAKE_BUILD_TYPE=Release "
    "--build-base {} "
    "--install-base {} "
    "--packages-up-to {} ",
    buildBase(ws_path).string(),
    installBase(ws_path).string(),
    pkg_name.value());

  // Add options
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

  // Build the Tobas project packages
  std::cout << "Executing \"" << build_cmd << "\" on " << exec_path.value() << "." << std::endl;
  if (!cmd_exec_.execute(build_cmd)) {
    error_msg_ = "Failed to build \"" + pkg_name.value() + "\":\n" + cmd_exec_.getOutput();
    return false;
  }

  return true;
}

bool Colcon::cleanWorkspace(const fs::path& ws_path)
{
  // Navigate to the colcon workspace
  if (chdir(ws_path.c_str()) != 0) {
    error_msg_ = "Failed to navigate to \"" + ws_path.string() + "\": " + linux::strError();
    return false;
  }

  // Clean the workspace
  if (!cmd_exec_.execute("colcon clean workspace -y")) {
    error_msg_ = "Failed to clean \"" + ws_path.string() + "\":\n" + cmd_exec_.getOutput();
    return false;
  }

  return true;
}

const std::string& Colcon::errorMessage() const
{
  return error_msg_;
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
