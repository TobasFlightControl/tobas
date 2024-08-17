#pragma once

#include <tobas_node/node.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>
#include <tobas_msgs/msg/battery.hpp>
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
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;

  rclcpp::Time last_cmd_time_;
  bool is_armed_ = false;
  bool is_activated_ = false;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;

  // PubSub
  PublisherPtr<> throttles_pub_;
  PublisherPtr<> arming_pub_;
  SubscriberPtr<> tar_speeds_sub_;
  SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;

  // Service
  ServicePtr<> get_arm_ss_;
  ServicePtr<> set_arm_ss_;
  rclcpp::ServiceClient enable_rcout_sc_;
  rclcpp::ServiceClient pre_arm_check_sc_;

  // Timer
  TimerPtr check_interval_timer_;

  bool armRotors();
  bool disarmRotors();
  bool enableRCOutputs(const bool& enable);
  bool preArmCheck();
  void setThrottleOnAllChannels(const double& throttle);
  void publishArming();

  void rotSpeedsCmdCb(const tobas_msgs::msg::RotorSpeeds::ConstSharedPtr& tar_speeds);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);

  bool getArmCb(tobas_msgs::GetArmRequest& req, tobas_msgs::GetArmResponse& res);
  bool setArmCb(tobas_msgs::SetArmRequest& req, tobas_msgs::SetArmResponse& res);

  void checkIntervalTimerCb();
};
}  // namespace tobas_rotor_controller
