#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/PreArmCheck.h>

namespace tobas_pre_arm_check
{
class PreArmCheckServer : public tobas::BaseNode
{
  static constexpr double kAttitudeThreshold = M_PI / 6;    // [rad/s]
  static constexpr double kHorPosStddevThreshold = 1.;      // [m]
  static constexpr double kVerPosStddevThreshold = 2.;      // [m]
  static constexpr double kRotStddevThreshold = M_PI / 24;  // [rad]
  static constexpr double kVelStddevThreshold = 0.3;        // [m/s]

  static constexpr double kPreArmCheckTimerRate = 1.;  // [s]

  using self = PreArmCheckServer;
  using super = tobas::BaseNode;

public:
  explicit PreArmCheckServer(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;

  tobas_msgs::BatteryConstPtr battery_;
  tobas_msgs::OdometryConstPtr odom_;

  double roll_, pitch_, yaw_;
  Eigen::Matrix3d cov_;
  tobas_msgs::PreArmCheck pre_arm_check_;

  ros::Publisher pre_arm_check_pub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber odom_sub_;

  ros::ServiceServer pre_arm_check_ss_;

  ros::Timer pre_arm_check_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void odomCb(const tobas_msgs::OdometryConstPtr& odom);

  bool preArmCheckSrvCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void preArmCheckTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_pre_arm_check
