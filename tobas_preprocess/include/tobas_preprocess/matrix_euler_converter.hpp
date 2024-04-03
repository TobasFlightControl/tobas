#pragma once

#include <ros/ros.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Odometry.h>

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
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  ros::Publisher euler_pub_;
  ros::Subscriber odom_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
};
}  // namespace tobas_preprocess
