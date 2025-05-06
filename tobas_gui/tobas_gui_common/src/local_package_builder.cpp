#include "tobas_gui_common/local_package_builder.hpp"

#include <unistd.h>

#include <format>
#include <iostream>

#include <tobas_constants/constants.hpp>
#include <tobas_ros2_tools/util.hpp>

#include "tobas_gui_common/package.hpp"

using namespace std;
namespace fs = filesystem;

namespace gui
{
namespace common
{
LocalPackageBuilder::LocalPackageBuilder()
{
}

bool LocalPackageBuilder::build(const fs::path& tbs_path)
{
  // ビルドできれば終了
  if (colconBuild(tbs_path)) {
    return true;
  }

  // ビルドできなければ一度ワークスペースを初期化
  if (!colconCleanWorkspace()) {
    return false;
  }

  // クリーンビルドできれば終了
  if (colconBuild(tbs_path)) {
    return true;
  }

  return false;
}

const string& LocalPackageBuilder::getOutput() const
{
  return command_executor_.getOutput();
}

bool LocalPackageBuilder::colconBuild(const std::filesystem::path& tbs_path)
{
  // Navigate to the Tobas package
  if (chdir(tbs_path.c_str()) != 0) {
    cerr << "Failed to navigate to \"" << tbs_path << "\"." << endl;
    return false;
  }

  // Get paths needed for building
  const auto ws_path = ros2::expandUser(tobas::kColconWSPathHome);
  const auto build_path = ws_path / "build";
  const auto install_path = ws_path / "install";
  const auto log_path = ws_path / "log";

  // Specify the log directory
  if (setenv("COLCON_LOG_PATH", log_path.c_str(), 1) != 0) {
    cerr << "Failed to set the colcon log directory path to \"" << log_path << "\"." << endl;
    return false;
  }

  // Create build command
  const auto meta_name = getTBSMetaName(tbs_path);
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

  // Build Tobas package
  cout << "Executing \"" << build_cmd << "\" on " << tbs_path << "." << endl;
  if (!command_executor_.execute(build_cmd)) {
    cerr << "Failed to build \"" << meta_name << "\"." << endl;
    return false;
  }

  return true;
}

bool LocalPackageBuilder::colconCleanWorkspace()
{
  // Navigate to the colcon workspace
  const auto ws_path = ros2::expandUser(tobas::kColconWSPathHome);
  if (chdir(ws_path.c_str()) != 0) {
    cerr << "Failed to navigate to \"" << ws_path << "\"." << endl;
    return false;
  }

  // Clean workspace
  if (!command_executor_.execute("colcon clean workspace -y")) {
    cerr << "Failed to clean " << ws_path << "." << endl;
    return false;
  }

  return true;
}
}  // namespace common
}  // namespace gui
