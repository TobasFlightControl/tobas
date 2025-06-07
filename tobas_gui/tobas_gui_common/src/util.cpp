#include "tobas_gui_common/util.hpp"

#include <format>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace common
{
fs::path getIconPath(const std::string& color)
{
  const fs::path pkg_path(ament_index_cpp::get_package_share_directory("tobas_gui_common"));
  return pkg_path / std::format("resources/icon_{}.png", color);
}
}  // namespace common
}  // namespace gui
