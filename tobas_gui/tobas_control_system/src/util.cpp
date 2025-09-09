#include "tobas_control_system/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tobas_control_system/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace ctrl
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory(kPackageName);
}
}  // namespace ctrl
}  // namespace gui
