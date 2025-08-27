#pragma once

#include <rclcpp/time.hpp>

namespace ros2
{
class RateManager
{
public:
  using SharedPtr = std::shared_ptr<RateManager>;

  explicit RateManager(double update_rate);

  void reset();

  /* 実行可能な周期ならばtrue． */
  bool update(const rclcpp::Time& time);

private:
  const rclcpp::Duration interval_;
  rclcpp::Time t_next_;
  bool is_first_update_ = true;
};
}  // namespace ros2
