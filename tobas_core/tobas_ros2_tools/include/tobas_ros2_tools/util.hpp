#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace ros2
{
/* Vector3のL2ノルムを計算する． */
double norm(const geometry_msgs::msg::Vector3& v);

/* JointStateの各フィールドのサイズが合っているかを調べる． */
bool isFieldSizeMatch(const sensor_msgs::msg::JointState& js);

/* JointStateを初期化． */
void clear(sensor_msgs::msg::JointState& js);

/* JointStateをリサイズ． */
void resize(sensor_msgs::msg::JointState& js, const size_t& size);

/* 条件が真になるまでspinしながら待機する． */
template <typename Lambda>
bool spinUntil(
  rclcpp::Node::SharedPtr node,
  const Lambda& cond,
  const double& timeout = std::numeric_limits<double>::max(),
  const double& spin_rate = 1000.)
{
  assert(spin_rate > 0);

  const auto start_time = node->get_clock()->now();
  rclcpp::Rate rate(spin_rate);

  while (rclcpp::ok())
  {
    if (cond())
      return true;

    if ((node->get_clock()->now() - start_time).seconds() > timeout)
      return false;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  return false;
}
}  // namespace ros2
