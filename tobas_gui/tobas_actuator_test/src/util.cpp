#include "tobas_actuator_test/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace at
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory("tobas_actuator_test");
}
}  // namespace at
}  // namespace gui
}  // namespace tobas
