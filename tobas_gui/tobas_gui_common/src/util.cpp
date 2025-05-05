#include "../include/tobas_gui_common/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std;
namespace fs = filesystem;

namespace gui
{
namespace common
{
fs::path getIconPath()
{
  const fs::path pkg_path(ament_index_cpp::get_package_share_directory("tobas_gui_common"));
  return pkg_path / "resources/icon.png";
}
}  // namespace common
}  // namespace gui
