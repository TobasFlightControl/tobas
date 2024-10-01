#include <iostream>
#include <unistd.h>

#include <tobas_linux/core.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_gui_common/local_package_builder.hpp"
#include "../include/tobas_gui_common/constants.hpp"
#include "../include/tobas_gui_common/package.hpp"

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
  // Navigate to the Tobas package
  if (chdir(tbs_path.c_str()) != 0)
  {
    cerr << "Failed to navigate to \"" << tbs_path << "\"." << endl;
    return false;
  }

  // Get paths needed for building
  const auto ws_path = linux::expandUser(kColconWSPathPC);
  const auto build_path = ws_path / "build";
  const auto install_path = ws_path / "install";
  const auto log_path = ws_path / "log";

  // Specify the log directory
  if (setenv("COLCON_LOG_PATH", log_path.c_str(), 1) != 0)
  {
    cerr << "Failed to set the colcon log directory path to \"" << log_path << "\"." << endl;
    return false;
  }

  // Create build command
  const auto meta_name = getTBSMetaName(tbs_path);
  const auto command = format(
    "colcon build "
    "--symlink-install "
    "--parallel-workers $(nproc) "
    "--cmake-args -DCMAKE_BUILD_TYPE=Release "
    "--build-base {} "
    "--install-base {} "
    "--packages-up-to {} ",
    build_path.string(), install_path.string(), meta_name);

  // Build Tobas package
  cout << "Executing \"" << command << "\" on " << tbs_path << "." << endl;
  if (!command_executor_.execute(command))
  {
    cerr << "Failed to build \"" << meta_name << "\"." << endl;
    return false;
  }

  return true;
}

const string& LocalPackageBuilder::getOutput() const
{
  return command_executor_.getOutput();
}
}  // namespace common
}  // namespace gui
