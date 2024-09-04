#include <iostream>
#include <unistd.h>

#include <tobas_linux/core.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_tools/command.hpp"
#include "../include/tobas_tools/package.hpp"

using namespace std;
namespace fs = filesystem;

namespace tobas
{
bool buildTobasPackage(const fs::path& tbs_path)
{
  // Navigate to the Tobas package
  if (chdir(tbs_path.c_str()) != 0)
  {
    cerr << "Failed to navigate to \"" << tbs_path << "\"." << endl;
    return false;
  }

  // Get paths needed for building
  const fs::path ws_path = linux::expandUser(kColconWSPath);
  const auto build_path = ws_path / "build";
  const auto install_path = ws_path / "install";
  const auto log_path = ws_path / "log";

  // Specify the log directory
  if (setenv("COLCON_LOG_PATH", log_path.c_str(), 1) != 0)
  {
    cerr << "Failed to set the colcon log directory path to \"" << log_path << "\"." << endl;
    return false;
  }

  // Build the meta package
  const auto meta_name = getTBSMetaName(tbs_path);
  const auto command = format(
    "colcon build "
    "--symlink-install "
    "--parallel-workers $(nproc) "
    "--cmake-args -DCMAKE_BUILD_TYPE=Release "
    "--build-base {}"
    "--install-base {}"
    "--packages-up-to {}",
    build_path.string(), install_path.string(), meta_name);
  if (system(command.c_str()) != 0)
  {
    cerr << "Failed to build \"" << meta_name << "\"." << endl;
    return false;
  }

  return true;
}

bool sourceTobasPackage(const fs::path& tbs_path)
{
  constexpr char AMENT_PREFIX_PATH[] = "AMENT_PREFIX_PATH";

  // Get old paths
  const auto old_paths = getenv(AMENT_PREFIX_PATH);
  if (old_paths == nullptr)
  {
    cerr << "Failed to get \"" << AMENT_PREFIX_PATH << "\"." << endl;
    return false;
  }

  // Set new paths
  const auto config_path = getTBSConfigPath(tbs_path);
  const auto user_path = getTBSConfigPath(tbs_path);
  const auto new_paths = config_path.string() + ":" + user_path.string() + ":" + old_paths;
  if (setenv(AMENT_PREFIX_PATH, new_paths.c_str(), 1) != 0)
  {
    cerr << "Failed to set \"" << AMENT_PREFIX_PATH << "\"." << endl;
    return false;
  }

  return false;
}
}  // namespace tobas
