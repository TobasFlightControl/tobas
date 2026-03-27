#include "tobas_sensor_calibration/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace sc
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory("tobas_sensor_calibration");
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
