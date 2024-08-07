#pragma once

#include <rclcpp/rclcpp.hpp>
#include <actionlib/server/simple_action_server.h>

#include <tobas_drone_core/drone.hpp>
#include <tobas_node/node.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_calibration_msgs/EscCalibrationAction.h>

namespace tobas_calibration
{
class EscCalibrationRos : public tobas::BaseNode
{
  static constexpr char kActionName[] = "esc_calibration";

  static constexpr double kHighDuration = 3.;          // [s]
  static constexpr double kLowDuration = 5.;           // [s]
  static constexpr double kWaitForBatteryTopic = 0.1;  // [s]
  static constexpr double kTimeout = 30.;              // [s]
  static constexpr size_t kInterval = 10;              // [ms]
  static constexpr double kVoltageThreshold = 3.;      // [V]

  using super = tobas::BaseNode;
  using ActionType = tobas_calibration_msgs::EscCalibrationAction;
  using GoalType = tobas_calibration_msgs::EscCalibrationGoal;
  using ResultType = tobas_calibration_msgs::EscCalibrationResult;
  using FeedbackType = tobas_calibration_msgs::EscCalibrationFeedback;

public:
  explicit EscCalibrationRos(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  tobas::Drone drone_;
  ResultType result_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;

  PublisherPtr<> throttles_pub_;
  rclcpp::ServiceClient get_arm_sc_;
  rclcpp::ServiceClient enable_rcout_sc_;
  actionlib::SimpleActionServer<ActionType> as_;

  void sendMaximum();
  void sendMinimum();
  void setThrottle(const double& throttle);
  void setThrottleAndSleep(const double& throttle);
  bool checkDisarmed();
  bool enableRCOutput(bool enable);
  bool checkBatteryDisconnected();
  bool waitForBatteryConnection();

  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void executeCb(const GoalType::ConstSharedPtr& goal);
};
}  // namespace tobas_calibration
