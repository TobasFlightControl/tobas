#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../include/tobas_gui_common/util.hpp"
#include "../include/tobas_gui_common/package.hpp"

#define AMENT_PREFIX_PATH "AMENT_PREFIX_PATH"

using namespace std;
namespace fs = filesystem;

namespace gui
{
namespace common
{
fs::path getIconPath()
{
  const auto pkg_path = fs::path(ament_index_cpp::get_package_share_directory("tobas_gui_common"));
  return pkg_path / "resources/icon.png";
}

bool addAmentPrefixPath(const fs::path& path)
{
  const auto old_paths = getenv(AMENT_PREFIX_PATH);
  if (old_paths == nullptr)
  {
    cerr << "Failed to get environment variable \"" << AMENT_PREFIX_PATH << "\"." << endl;
    return false;
  }

  const auto new_paths = path.string() + ":" + old_paths;
  if (setenv(AMENT_PREFIX_PATH, new_paths.c_str(), 1) != 0)
  {
    cerr << "Failed to set environment variable \"" << AMENT_PREFIX_PATH << "\"." << endl;
    return false;
  }

  return true;
}
}  // namespace common
}  // namespace gui
