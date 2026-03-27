#include "tobas_ros2_tools/time.hpp"

#define BILLION 1'000'000'000

namespace ch = std::chrono;

namespace tobas
{
namespace ros2
{
void timeChronoToMsg(const ch::steady_clock::duration& c, builtin_interfaces::msg::Time& m)
{
  const auto nsec = c.count();
  assert(nsec >= 0);
  m.sec = nsec / BILLION;
  m.nanosec = nsec % BILLION;
}
}  // namespace ros2
}  // namespace tobas
