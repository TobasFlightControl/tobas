#include "tobas_ros2_tools/time.hpp"

#define BILLION 1'000'000'000

using namespace std;

namespace ros2
{
void timeChronoToMsg(const chrono::steady_clock::duration& c, builtin_interfaces::msg::Time& m)
{
  const auto nsec = c.count();
  assert(nsec >= 0);
  m.sec = nsec / BILLION;
  m.nanosec = nsec % BILLION;
}
}  // namespace ros2

rclcpp::Duration operator-(const builtin_interfaces::msg::Time& lhs, const builtin_interfaces::msg::Time& rhs)
{
  const auto lhs_ns = RCL_S_TO_NS(lhs.sec) + lhs.nanosec;
  const auto rhs_ns = RCL_S_TO_NS(rhs.sec) + rhs.nanosec;
  const auto dur_ns = lhs_ns - rhs_ns;
  return rclcpp::Duration::from_nanoseconds(dur_ns);
}
