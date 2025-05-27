#include "tobas_ros2_tools/rate_manager.hpp"

namespace ros2
{
RateManager::RateManager(double update_rate)
  : interval_(rclcpp::Duration::from_nanoseconds(1'000'000'000 / update_rate))
{
  assert(update_rate > 0.);
}

void RateManager::reset()
{
  is_first_update_ = true;
}

bool RateManager::update(const rclcpp::Time& time)
{
  if (is_first_update_) {
    is_first_update_ = false;
    t_next_ = time + interval_;
    return true;
  }

  if (time < t_next_) {
    return false;
  }
  else {
    t_next_ += interval_;
    return true;
  }
}
}  // namespace ros2
