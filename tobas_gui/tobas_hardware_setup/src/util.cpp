#include "tobas_hardware_setup/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tobas_hardware_setup/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace hw
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory(kPackageName);
}
}  // namespace hw
}  // namespace gui
