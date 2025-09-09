#include "tobas_actuator_test/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tobas_actuator_test/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace at
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory(kPackageName);
}
}  // namespace at
}  // namespace gui
