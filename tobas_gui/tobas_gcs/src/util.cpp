#include "tobas_gcs/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tobas_gcs/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace gcs
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory(kPackageName);
}

fs::path getResourceDir()
{
  return getPkgShareDir() / "resources";
}
}  // namespace gcs
}  // namespace gui
