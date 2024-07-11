#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_tools/drone.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_msgs/Battery.h>
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
    ros::NodeHandle& nh,
    ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;
  ResultType result_;
  tobas_msgs::BatteryConstPtr battery_;

  ros::Publisher pwms_pub_;
  ros::ServiceClient get_arm_sc_;
  ros::ServiceClient enable_pwm_sc_;
  actionlib::SimpleActionServer<ActionType> as_;

  void sendMaximum();
  void sendMinimum();
  void setPeriod(const double& period);
  void setPeriodAndSleep(const double& period);
  bool checkDisarmed();
  bool enablePWM();
  void disablePWM();
  bool checkBatteryDisconnected();
  bool waitForBatteryConnection();

  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void executeCb(const GoalType::ConstPtr& goal);
};
}  // namespace tobas_calibration
