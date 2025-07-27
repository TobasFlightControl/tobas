#include "tobas_gui_common/local_project_builder.hpp"

#include <unistd.h>

#include <format>
#include <iostream>

#include <tobas_constants/constants.hpp>
#include <tobas_ros2_tools/util.hpp>

#include "tobas_gui_common/path.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace common
{
LocalProjectBuilder::LocalProjectBuilder()
{
}

bool LocalProjectBuilder::build(const fs::path& proj_path)
{
  // ビルドできれば終了
  if (colconBuild(proj_path)) {
    return true;
  }

  // ビルドできなければ一度ワークスペースを初期化
  if (!colconCleanWorkspace()) {
    return false;
  }

  // クリーンビルドできれば終了
  if (colconBuild(proj_path)) {
    return true;
  }

  return false;
}

const std::string& LocalProjectBuilder::getOutput() const
{
  return command_executor_.getOutput();
}

bool LocalProjectBuilder::colconBuild(const fs::path& proj_path)
{
  // Navigate to the Tobas project directory
  if (chdir(proj_path.c_str()) != 0) {
    std::cerr << "Failed to navigate to \"" << proj_path << "\"." << std::endl;
    return false;
  }

  // Get paths needed for building
  const auto ws_path = ros2::expandUser(tobas::kColconWSPathHome);
  const auto build_path = ws_path / "build";
  const auto install_path = ws_path / "install";
  const auto log_path = ws_path / "log";

  // Specify the log directory
  if (setenv("COLCON_LOG_PATH", log_path.c_str(), 1) != 0) {
    std::cerr << "Failed to set the colcon log directory path to \"" << log_path << "\"." << std::endl;
    return false;
  }

  // Create build command
  const auto meta_name = getProjMetaPkgName(proj_path);
  const auto build_cmd = format(
    "colcon build "
    "--merge-install "
    "--parallel-workers $(nproc) "
    "--cmake-args -DCMAKE_BUILD_TYPE=Release "
    "--build-base {} "
    "--install-base {} "
    "--packages-up-to {} ",
    build_path.string(),
    install_path.string(),
    meta_name);

  // Build Tobas project packages
  std::cout << "Executing \"" << build_cmd << "\" on " << proj_path << "." << std::endl;
  if (!command_executor_.execute(build_cmd)) {
    std::cerr << "Failed to build \"" << meta_name << "\"." << std::endl;
    return false;
  }

  return true;
}

bool LocalProjectBuilder::colconCleanWorkspace()
{
  // Navigate to the colcon workspace
  const auto ws_path = ros2::expandUser(tobas::kColconWSPathHome);
  if (chdir(ws_path.c_str()) != 0) {
    std::cerr << "Failed to navigate to \"" << ws_path << "\"." << std::endl;
    return false;
  }

  // Clean workspace
  if (!command_executor_.execute("colcon clean workspace -y")) {
    std::cerr << "Failed to clean " << ws_path << "." << std::endl;
    return false;
  }

  return true;
}
}  // namespace common
}  // namespace gui
