#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/Odometry.h>

namespace tobas_pre_arm_check
{
class PreArmCheckServer : public tobas::BaseNode
{
  static constexpr double kGyroNormThreshold = M_PI / 6;    // [rad/s]
  static constexpr double kHorPosStddevThreshold = 1.0;     // [m]
  static constexpr double kVerPosStddevThreshold = 2.0;     // [m]
  static constexpr double kRotStddevThreshold = M_PI / 24;  // [rad]
  static constexpr double kVelStddevThreshold = 0.3;        // [m/s]

  using self = PreArmCheckServer;
  using super = tobas::BaseNode;

public:
  explicit PreArmCheckServer(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;

  std_msgs::BoolConstPtr arming_ = boost::make_shared<std_msgs::Bool>();  // デフォルトでFalse
  tobas_msgs::BatteryConstPtr battery_;
  tobas_msgs::OdometryConstPtr odom_;

  Eigen::Matrix3d cov_;

  ros::Subscriber arming_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber odom_sub_;

  ros::ServiceServer ss_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void reset();

  void armingCb(const std_msgs::BoolConstPtr& arming);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void odomCb(const tobas_msgs::OdometryConstPtr& odom);

  bool executeCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
};
}  // namespace tobas_pre_arm_check
