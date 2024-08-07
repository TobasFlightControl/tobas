#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_node/node.hpp>
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
  PublisherPtr<> euler_pub_;
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
};
}  // namespace tobas_preprocess
