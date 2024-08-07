#include <tobas_constants/constants.hpp>
#include <tobas_kdl_msgs/EulerStamped.h>

#include "../include/tobas_preprocess/matrix_euler_converter.hpp"

using namespace std;

namespace tobas_preprocess
{
MatrixEulerConverter::MatrixEulerConverter(const rclcpp::NodeOptions& options)
  : super(node, pnh, name)
{
  euler_pub_ = createPublisher<tobas_kdl_msgs::EulerStamped>(tobas::kEulerTopic);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
}

void MatrixEulerConverter::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  const auto euler =std::make_unique<tobas_kdl_msgs::EulerStamped>();
  euler->header = odom->header;
  odom->frame.M.getRPY(euler->euler.roll, euler->euler.pitch, euler->euler.yaw);
  euler_pub_->publish(euler);
}
}  // namespace tobas_preprocess
