#include "tobas_control_system/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace ctrl
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory("tobas_control_system");
}
}  // namespace ctrl
}  // namespace gui
