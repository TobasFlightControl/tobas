#include "tobas_urdf_builder_plugin/utils/time.hpp"

#include <chrono>

namespace gui
{
namespace ub
{
namespace utils
{
int timeNowMilliseconds()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now);
  return static_cast<int>(ms.count());
}
}  // namespace utils
}  // namespace ub
}  // namespace gui
