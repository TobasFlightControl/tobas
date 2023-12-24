#include <chrono>

#include "../../include/urdf_builder/utils/time.hpp"

namespace urdf_builder
{
namespace utils
{
int timeNowMilliseconds()
{
  using namespace std::chrono;
  const auto now = system_clock::now().time_since_epoch();
  const auto ms = duration_cast<milliseconds>(now);
  return static_cast<int>(ms.count());
}
}  // namespace utils
}  // namespace urdf_builder
