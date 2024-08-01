#pragma once

#include <std_srvs/srv/trigger.hpp>

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
  static constexpr double kOdomCallbackInterval = 0.1;    // [s]
  static constexpr double kPosDriftCheckTimeWindow = 5.;  // [s]
  static constexpr double kPosDriftThresh = 1.;           // [m]
  static constexpr double kAttitudeThresh = M_PI / 6;     // [rad/s]
  static constexpr double kHorPosStddevThresh = 1.;       // [m]
  static constexpr double kVerPosStddevThresh = 2.;       // [m]
  static constexpr double kRotStddevThresh = M_PI / 24;   // [rad]
  static constexpr double kVelStddevThresh = 0.3;         // [m/s]
  static constexpr double kPreArmCheckTimerRate = 1.;     // [s]

  using self = PreArmCheckServer;
  using super = tobas::BaseNode;

public:
  explicit PreArmCheckServer(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  tobas::Drone drone_;

  tobas_msgs::BatteryConstPtr battery_;
  tobas_msgs::OdometryConstPtr odom_;

  std::array<tobas_std::TimestampedBufferDouble, 3> pos_buf_;
  double roll_, pitch_, yaw_;
  tobas_msgs::PreArmCheck pre_arm_check_;

  rclcpp::Publisher pre_arm_check_pub_;
  rclcpp::Subscriber battery_sub_;
  rclcpp::Subscriber odom_sub_;
  rclcpp::ServiceServer pre_arm_check_ss_;
  rclcpp::Timer pre_arm_check_timer_;

  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void odomCb(const tobas_msgs::OdometryConstPtr& odom);

  bool preArmCheckSrvCb(std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res);
  void preArmCheckTimerCb(const rclcpp::TimerEvent& event);
};
}  // namespace tobas_pre_arm_check
