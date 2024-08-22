#include <chrono>

#include "../../include/urdf_builder/utils/time.hpp"

using namespace std;

namespace urdf_builder
{
namespace utils
{
int timeNowMilliseconds()
{
  const auto now = chrono::system_clock::now().time_since_epoch();
  const auto ms = chrono::duration_cast<chrono::milliseconds>(now);
  return static_cast<int>(ms.count());
}
}  // namespace utils
}  // namespace urdf_builder
