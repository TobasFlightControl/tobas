#pragma once

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/GetArm.h>
#include <tobas_msgs/SetArm.h>

namespace tobas_rotor_controller
{
class RotorController : public tobas::BaseNode
{
  static constexpr double kDisarmThrottle = -0.1;
  static constexpr double kDisarmDuration = 3.;          // [s]
  static constexpr double kDisarmInterval = 0.1;         // [s]
  static constexpr size_t kCheckIntervalTimerRate = 10;  // [Hz]

  using self = RotorController;
  using super = tobas::BaseNode;

public:
  explicit RotorController(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  tobas::Drone drone_;

  rclcpp::Time last_cmd_time_;
  bool is_armed_ = false;
  bool is_activated_ = false;
  tobas_msgs::BatteryConstPtr battery_;

  // PubSub
  rclcpp::Publisher throttles_pub_;
  rclcpp::Publisher arming_pub_;
  rclcpp::Subscriber tar_speeds_sub_;
  rclcpp::Subscriber battery_sub_;

  // Service
  rclcpp::ServiceServer get_arm_ss_;
  rclcpp::ServiceServer set_arm_ss_;
  rclcpp::ServiceClient enable_rcout_sc_;
  rclcpp::ServiceClient pre_arm_check_sc_;

  // Timer
  rclcpp::Timer check_interval_timer_;

  bool armRotors();
  bool disarmRotors();
  bool enableRCOutputs(const bool& enable);
  bool preArmCheck();
  void setThrottleOnAllChannels(const double& throttle);
  void publishArming();

  void rotSpeedsCmdCb(const tobas_msgs::RotorSpeedsConstPtr& tar_speeds);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);

  bool getArmCb(tobas_msgs::GetArmRequest& req, tobas_msgs::GetArmResponse& res);
  bool setArmCb(tobas_msgs::SetArmRequest& req, tobas_msgs::SetArmResponse& res);

  void checkIntervalTimerCb(const rclcpp::TimerEvent& event);
};
}  // namespace tobas_rotor_controller
