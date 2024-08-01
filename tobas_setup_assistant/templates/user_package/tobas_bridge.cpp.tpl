#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/{{ user_pkg_name }}/tobas_bridge.hpp"

#define MAIN_TIMER_FREQ 100

TobasBridge::TobasBridge() : node_(node), pnh_(pnh)
{
  /* Register publishers */
  // js_pub_ = node_.advertise<sensor_msgs::msg::JointState>(tobas::kJointStatesTopic, 1);

  /* Register subscribers */
  // tar_pos_sub_ = node_.subscribe(tobas::kJointPositionsCmdTopic, 1, &self::jntPosCmdCb, this);
  // tar_vel_sub_ = node_.subscribe(tobas::kJointVelocitiesCmdTopic, 1, &self::jntVelCmdCb, this);
  // tar_eff_sub_ = node_.subscribe(tobas::kJointEffortsCmdTopic, 1, &self::jntEffCmdCb, this);

  /* Create main timer */
  // main_timer_ = node_.createTimer(MAIN_TIMER_FREQ, &self::mainTimerCb, this);
}

void TobasBridge::jntPosCmdCb(const tobas_msgs::JointCommandArrayConstPtr& tar_pos)
{
  /* TODO */
}

void TobasBridge::jntVelCmdCb(const tobas_msgs::JointCommandArrayConstPtr& tar_vel)
{
  /* TODO */
}

void TobasBridge::jntEffCmdCb(const tobas_msgs::JointCommandArrayConstPtr& tar_eff)
{
  /* TODO */
}

void TobasBridge::mainTimerCb(const rclcpp::TimerEvent& event)
{
  /* TODO */
}
