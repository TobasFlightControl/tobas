#include <sensor_msgs/JointState.h>

#include <tobas_tools/constants.hpp>

#include "../include/{{ user_pkg_name }}/tobas_bridge.hpp"

#define MAIN_TIMER_FREQ 100

TobasBridge::TobasBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
{
  /* Register publishers */
  // js_pub_ = nh_.advertise<sensor_msgs::JointState>(tobas::kJointStatesTopic, 1);

  /* Register subscribers */
  // tar_pos_sub_ = nh_.subscribe(tobas::kJointPositionsCmdTopic, 1, &self::jntPosCmdCb, this);
  // tar_vel_sub_ = nh_.subscribe(tobas::kJointVelocitiesCmdTopic, 1, &self::jntVelCmdCb, this);
  // tar_eff_sub_ = nh_.subscribe(tobas::kJointEffortsCmdTopic, 1, &self::jntEffCmdCb, this);

  /* Create main timer */
  // main_timer_ = nh_.createTimer(MAIN_TIMER_FREQ, &self::mainTimerCb, this);
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

void TobasBridge::mainTimerCb(const ros::TimerEvent& event)
{
  /* TODO */
}
