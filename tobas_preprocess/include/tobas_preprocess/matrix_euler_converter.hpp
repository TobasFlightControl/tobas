#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Odometry.hpp>

namespace tobas_preprocess
{
/**
 * @brief オドメトリから得られた姿勢をオイラー角に変換して発行する．
 */
class MatrixEulerConverter : public tobas::BaseNode
{
  using self = MatrixEulerConverter;
  using super = tobas::BaseNode;

public:
  explicit MatrixEulerConverter(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  rclcpp::Publisher euler_pub_;
  rclcpp::Subscriber odom_sub_;

  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
};
}  // namespace tobas_preprocess
