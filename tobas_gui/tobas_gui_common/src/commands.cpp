#include "tobas_gui_common/command.hpp"

#include <unistd.h>

#include <iostream>

#include "tobas_gui_common/package.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace common
{
bool sourceTobasProject(const fs::path& tbs_path)
{
  constexpr char AMENT_PREFIX_PATH[] = "AMENT_PREFIX_PATH";

  // Get old paths
  const auto old_paths = getenv(AMENT_PREFIX_PATH);
  if (!old_paths) {
    std::cerr << "Failed to get \"" << AMENT_PREFIX_PATH << "\"." << std::endl;
    return false;
  }

  // Set new paths
  const auto config_path = getTBSConfigPath(tbs_path);
  const auto user_path = getTBSConfigPath(tbs_path);
  const auto new_paths = config_path.string() + ":" + user_path.string() + ":" + old_paths;
  if (setenv(AMENT_PREFIX_PATH, new_paths.c_str(), 1) != 0) {
    std::cerr << "Failed to set \"" << AMENT_PREFIX_PATH << "\"." << std::endl;
    return false;
  }

  return false;
}
}  // namespace common
}  // namespace gui
