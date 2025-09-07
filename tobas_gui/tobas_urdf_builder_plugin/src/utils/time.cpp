#include "tobas_urdf_builder_plugin/utils/time.hpp"

#include <chrono>

namespace ch = std::chrono;

namespace gui
{
namespace ub
{
namespace utils
{
int timeNowMilliseconds()
{
  const auto now = ch::system_clock::now().time_since_epoch();
  const auto ms = ch::duration_cast<ch::milliseconds>(now);
  return static_cast<int>(ms.count());
}
}  // namespace utils
}  // namespace ub
}  // namespace gui
