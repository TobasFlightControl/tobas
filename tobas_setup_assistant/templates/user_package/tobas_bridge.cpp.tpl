#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_constants/constants.hpp>

#include "../include/{{ user_pkg_name }}/tobas_bridge.hpp"

#define MAIN_TIMER_FREQ 100

TobasBridge::TobasBridge() : node_(node), pnh_(pnh)
{
  /* Register publishers */
  // js_pub_ = createPublisher<sensor_msgs::msg::JointState>(tobas::kJointStatesTopic);

  /* Register subscribers */
  // tar_pos_sub_ = createSubscriber(tobas::kJointPositionsCmdTopic, &self::jntPosCmdCb, this);
  // tar_vel_sub_ = createSubscriber(tobas::kJointVelocitiesCmdTopic, &self::jntVelCmdCb, this);
  // tar_eff_sub_ = createSubscriber(tobas::kJointEffortsCmdTopic, &self::jntEffCmdCb, this);

  /* Create main timer */
  // main_timer_ = node_.createTimer(MAIN_TIMER_FREQ, &self::mainTimerCb, this);
}

void TobasBridge::jntPosCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& tar_pos)
{
  /* TODO */
}

void TobasBridge::jntVelCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& tar_vel)
{
  /* TODO */
}

void TobasBridge::jntEffCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& tar_eff)
{
  /* TODO */
}

void TobasBridge::mainTimerCb(const rclcpp::TimerEvent& event)
{
  /* TODO */
}
