#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <tobas_std_tools/first_order_filter.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/GetArm.h>
#include <tobas_msgs/SetArm.h>

namespace tobas_navio_ros
{
class MotorsHandler : public tobas::BaseNode
{
  static constexpr double kBLHeliClosedLoopLowRangeMaxERPM = 50000;
  static constexpr double kBLHeliClosedLoopMidRangeMaxERPM = 100000;
  static constexpr double kBLHeliClosedLoopHighRangeMaxERPM = 200000;

  static constexpr size_t kCheckIntervalTimerRate = 10;  // [Hz]
  static constexpr double kSetupPwmRetryInterval = 1.;   // [s]

  using self = MotorsHandler;
  using super = tobas::BaseNode;

public:
  explicit MotorsHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;

  ros::Time last_cmd_time_;
  bool is_armed_ = false;
  bool is_activated_ = false;
  tobas_msgs::BatteryConstPtr battery_;
  tobas_std::FirstOrderFilter<double> latency_filter_;

  // PubSub
  ros::Publisher pwms_pub_;
  ros::Publisher latency_pub_;
  ros::Publisher arming_pub_;
  ros::Subscriber tar_speeds_sub_;
  ros::Subscriber battery_sub_;

  // Service
  ros::ServiceServer get_arm_ss_;
  ros::ServiceServer set_arm_ss_;
  ros::ServiceClient enable_pwm_sc_;

  // Timer
  ros::Timer check_interval_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool armRotors();
  bool disarmRotors();
  bool enablePwms(const bool& enable);
  void setPeriodOnAllChannels(const double& period);
  void publishArming();

  void rotSpeedsCmdCb(const tobas_msgs::RotorSpeedsConstPtr& tar_speeds);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);

  bool getArmCb(tobas_msgs::GetArmRequest& req, tobas_msgs::GetArmResponse& res);
  bool setArmCb(tobas_msgs::SetArmRequest& req, tobas_msgs::SetArmResponse& res);

  void checkIntervalTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros
