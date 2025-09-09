#include "tobas_sensor_calibration/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tobas_sensor_calibration/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sc
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory(kPackageName);
}
}  // namespace sc
}  // namespace gui
