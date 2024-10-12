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
}  // namespace ros2
