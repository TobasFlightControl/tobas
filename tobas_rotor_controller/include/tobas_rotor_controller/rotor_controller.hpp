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
    ros::NodeHandle& nh,
    ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;

  ros::Time last_cmd_time_;
  bool is_armed_ = false;
  bool is_activated_ = false;
  tobas_msgs::BatteryConstPtr battery_;

  // PubSub
  ros::Publisher throttles_pub_;
  ros::Publisher arming_pub_;
  ros::Subscriber tar_speeds_sub_;
  ros::Subscriber battery_sub_;

  // Service
  ros::ServiceServer get_arm_ss_;
  ros::ServiceServer set_arm_ss_;
  ros::ServiceClient enable_pwm_sc_;
  ros::ServiceClient pre_arm_check_sc_;

  // Timer
  ros::Timer check_interval_timer_;

  bool armRotors();
  bool disarmRotors();
  bool enablePwms(const bool& enable);
  bool preArmCheck();
  void setThrottleOnAllChannels(const double& throttle);
  void publishArming();

  void rotSpeedsCmdCb(const tobas_msgs::RotorSpeedsConstPtr& tar_speeds);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);

  bool getArmCb(tobas_msgs::GetArmRequest& req, tobas_msgs::GetArmResponse& res);
  bool setArmCb(tobas_msgs::SetArmRequest& req, tobas_msgs::SetArmResponse& res);

  void checkIntervalTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_rotor_controller
