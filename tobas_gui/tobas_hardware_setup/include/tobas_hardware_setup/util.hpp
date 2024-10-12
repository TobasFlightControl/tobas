#pragma once

#include <rclcpp/rclcpp.hpp>

/**
 * @brief 条件が真になるまで待機する．
 * @note スピンはしないため，条件式に含まれる変数は別スレッドで更新する必要がある．
 */
template <typename Lambda>
bool sleepUntil(
  rclcpp::Node::SharedPtr node,
  const Lambda& cond,
  const double& timeout,
  const double& check_rate = 100.)
{
  assert(check_rate > 0);

  const auto start_time = node->get_clock()->now();
  rclcpp::Rate rate(check_rate, node->get_clock());

  while (rclcpp::ok())
  {
    if (cond())
      return true;

    if ((node->get_clock()->now() - start_time).seconds() > timeout)
      return false;

    rate.sleep();
  }

  return false;
}
