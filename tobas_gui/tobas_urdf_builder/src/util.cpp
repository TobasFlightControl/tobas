#include "tobas_urdf_builder/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace ub
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory("tobas_urdf_builder");
}
}  // namespace ub
}  // namespace gui
