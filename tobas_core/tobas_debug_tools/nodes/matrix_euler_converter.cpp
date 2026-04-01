// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_kdl_msgs_adapter/euler_stamped.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

namespace tobas
{
/**
 * @brief オドメトリから得られた姿勢をオイラー角に変換して発行する．
 */
class MatrixEulerConverterNode : public BaseNode
{
  using self = MatrixEulerConverterNode;
  using super = BaseNode;

public:
  explicit MatrixEulerConverterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<tobas_kdl_msgs::EulerStamped> euler_pub_;
  ros2::SubscriberPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_sub_;

  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom);
};

MatrixEulerConverterNode::MatrixEulerConverterNode(const rclcpp::NodeOptions& options)
  : super("matrix_euler_converter", nodeOptions_Default(options))
{
  euler_pub_ = createPublisher<tobas_kdl_msgs::EulerStamped>("euler");
  odom_sub_ = createSubscriber(topic::kOdometry, &self::odomCb, this);
}

void MatrixEulerConverterNode::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom)
{
  auto euler = std::make_unique<tobas_kdl_msgs::EulerStamped>();
  euler->header = odom->header;
  odom->odom.odom.frame.M.getRPY(euler->euler.roll, euler->euler.pitch, euler->euler.yaw);
  euler_pub_->publish(std::move(euler));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::MatrixEulerConverterNode)
