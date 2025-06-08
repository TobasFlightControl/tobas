#include "tobas_urdf_builder_plugin/utils/time.hpp"

#include <chrono>

using namespace std;

namespace gui
{
namespace ub
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
}  // namespace ub
}  // namespace gui
