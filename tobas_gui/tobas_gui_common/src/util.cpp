#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../include/tobas_gui_common/util.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace common
{
fs::path getIconPath()
{
  const auto pkg_path = fs::path(ament_index_cpp::get_package_share_directory("tobas_gui_common"));
  return pkg_path / "resources/icon.png";
}
}  // namespace common
}  // namespace gui
