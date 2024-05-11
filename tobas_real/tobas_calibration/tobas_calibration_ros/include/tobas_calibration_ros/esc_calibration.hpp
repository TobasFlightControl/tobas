#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_navio_core/adc.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_calibration_msgs/EscCalibrationAction.h>

namespace tobas_calibration
{
class EscCalibrationRos : public tobas::BaseNode
{
  static constexpr char kActionName[] = "esc_calibration";

  static constexpr double kSleepHigh = 3.;     // [s]
  static constexpr double kSleepLow = 5.;      // [s]
  static constexpr double kTimeout = 30.;      // [s]
  static constexpr useconds_t kInterval = 10;  // [ms]
  static constexpr int kA2ValueThreshold = 300;

  using super = tobas::BaseNode;
  using ActionType = tobas_calibration_msgs::EscCalibrationAction;
  using GoalType = tobas_calibration_msgs::EscCalibrationGoal;
  using ResultType = tobas_calibration_msgs::EscCalibrationResult;
  using FeedbackType = tobas_calibration_msgs::EscCalibrationFeedback;

public:
  explicit EscCalibrationRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  navio::ADC adc_;
  tobas::Drone drone_;
  ResultType result_;

  ros::Publisher pwms_pub_;
  ros::ServiceClient get_arm_sc_;
  ros::ServiceClient enable_pwm_sc_;
  actionlib::SimpleActionServer<ActionType> as_;

  void sendMaximum();
  void sendMinimum();

  void setPeriod(const double& period);
  void setPeriodAndSleep(const double& period);
  bool isBatteryConnected();
  bool checkDisarmed();
  bool enablePWM();
  void disablePWM();

  void executeCb(const GoalType::ConstPtr& goal);
};
}  // namespace tobas_calibration
