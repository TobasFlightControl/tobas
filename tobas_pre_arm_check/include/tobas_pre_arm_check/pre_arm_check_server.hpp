#pragma once

#include <std_srvs/srv/trigger.hpp>

#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_node/node.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/Odometry.hpp>
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
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;

  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;

  std::array<tobas_std::TimestampedBufferDouble, 3> pos_buf_;
  double roll_, pitch_, yaw_;
  tobas_msgs::PreArmCheck pre_arm_check_;

  PublisherPtr<> pre_arm_check_pub_;
  SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ServicePtr<> pre_arm_check_ss_;
  rclcpp::Timer pre_arm_check_timer_;

  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);

  bool preArmCheckSrvCb(std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res);
  void preArmCheckTimerCb();
};
}  // namespace tobas_pre_arm_check
