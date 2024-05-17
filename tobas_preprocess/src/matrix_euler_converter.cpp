#include <tobas_tools/constants.hpp>
#include <tobas_kdl_msgs/EulerStamped.h>

#include "../include/tobas_preprocess/matrix_euler_converter.hpp"

using namespace std;

namespace tobas_preprocess
{
MatrixEulerConverter::MatrixEulerConverter(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  euler_pub_ = nh_.advertise<tobas_kdl_msgs::EulerStamped>(tobas::kEulerTopic, 1);
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
}

void MatrixEulerConverter::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  const auto euler = boost::make_shared<tobas_kdl_msgs::EulerStamped>();
  euler->header = odom->header;
  odom->frame.M.getRPY(euler->euler.roll, euler->euler.pitch, euler->euler.yaw);
  euler_pub_.publish(euler);
}
}  // namespace tobas_preprocess
