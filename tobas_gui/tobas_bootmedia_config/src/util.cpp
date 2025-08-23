#include "tobas_bootmedia_config/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tobas_bootmedia_config/constants.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace bm
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory(kPackageName);
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
