// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/utilities.hpp>

#include <tobas_node/node.hpp>
#include <tobas_tools/util.hpp>

#include <tobas_command_msgs/msg/accel.hpp>
#include <tobas_command_msgs/msg/accel_pitch_yaw.hpp>
#include <tobas_command_msgs/msg/accel_yaw.hpp>
#include <tobas_command_msgs/msg/angle.hpp>
#include <tobas_command_msgs/msg/angle_throttle.hpp>
#include <tobas_command_msgs/msg/pos_vel_acc.hpp>
#include <tobas_command_msgs/msg/pos_vel_acc_pitch_yaw.hpp>
#include <tobas_command_msgs/msg/pos_vel_acc_yaw.hpp>
#include <tobas_command_msgs/msg/rate.hpp>
#include <tobas_command_msgs/msg/rate_throttle.hpp>
#include <tobas_command_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_kdl_msgs/msg/accel_stamped.hpp>
#include <tobas_kdl_msgs/msg/frame_with_covariance_stamped.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>

namespace tobas
{
class TopicInterfaceIfaceToLogicNode : public BaseNode
{
  using super = BaseNode;

public:
  explicit TopicInterfaceIfaceToLogicNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  template <typename MsgType>
  void addTopic(
    const std::string& _sub_topic,
    const std::string& _pub_topic,
    bool _latch = ros2::qos::kDefaultLatch,
    bool _reliable = ros2::qos::kDefaultReliable,
    size_t _queue_size = ros2::qos::kDefaultQueueSize);
};

TopicInterfaceIfaceToLogicNode::TopicInterfaceIfaceToLogicNode(const rclcpp::NodeOptions& _options)
  : super("topic_interface_iface_to_logic", nodeOptions_Default(_options))
{
  addTopic<tobas_kdl_msgs::msg::FrameWithCovarianceStamped>(topic::kExternalPose, topic::kExternalPose);
  addTopic<tobas_kdl_msgs::msg::AccelStamped>(topic::kAdditionalAccel, topic::kAdditionalAccel);
  addTopic<tobas_msgs::msg::RotorSpeedArray>(topic::kRotorSpeedsCmd, topic::kRotorSpeedsCmd);
  addTopic<tobas_msgs::msg::IcePropulsionSystemCommand>(topic::kIcePropulsionSystemCmd, topic::kIcePropulsionSystemCmd);
  addTopic<tobas_msgs::msg::PwmArray>(topic::kPwmCmd, topic::kPwmCmd);
  addTopic<tobas_command_msgs::msg::Rate>(topic::kRateCmd, topic::kRateCmd);
  addTopic<tobas_command_msgs::msg::RateThrottle>(topic::kRateThrotCmd, topic::kRateThrotCmd);
  addTopic<tobas_command_msgs::msg::Angle>(topic::kAngleCmd, topic::kAngleCmd);
  addTopic<tobas_command_msgs::msg::AngleThrottle>(topic::kAngleThrotCmd, topic::kAngleThrotCmd);
  addTopic<tobas_command_msgs::msg::Accel>(topic::kAccelCmd, topic::kAccelCmd);
  addTopic<tobas_command_msgs::msg::AccelYaw>(topic::kAccelYawCmd, topic::kAccelYawCmd);
  addTopic<tobas_command_msgs::msg::AccelPitchYaw>(topic::kAccelPitchYawCmd, topic::kAccelPitchYawCmd);
  addTopic<tobas_command_msgs::msg::PosVelAcc>(topic::kPosVelAccCmd, topic::kPosVelAccCmd);
  addTopic<tobas_command_msgs::msg::PosVelAccYaw>(topic::kPosVelAccYawCmd, topic::kPosVelAccYawCmd);
  addTopic<tobas_command_msgs::msg::PosVelAccPitchYaw>(topic::kPosVelAccPitchYawCmd, topic::kPosVelAccPitchYawCmd);
  addTopic<tobas_command_msgs::msg::SpeedRollDeltaPitch>(topic::kSpeedRollDpitchCmd, topic::kSpeedRollDpitchCmd);
  addTopic<tobas_msgs::msg::JointCommandArray>(topic::kJointPosCmd, topic::kJointPosCmd);
  addTopic<tobas_msgs::msg::JointCommandArray>(topic::kJointVelCmd, topic::kJointVelCmd);
  addTopic<tobas_msgs::msg::JointCommandArray>(topic::kJointEffCmd, topic::kJointEffCmd);
}

template <typename MsgType>
void TopicInterfaceIfaceToLogicNode::addTopic(
  const std::string& _sub_topic,
  const std::string& _pub_topic,
  bool _latch,
  bool _reliable,
  size_t _queue_size)
{
  const ros2::qos::QoS qos(_latch, _reliable, _queue_size);
  const auto publisher = create_publisher<MsgType>(_pub_topic, qos);

  const auto callback = [publisher](const typename MsgType::ConstSharedPtr& _msg)
  {
    // The standalone default disables intra-process delivery, avoiding an extra rclcpp copy here.
    publisher->publish(*_msg);
  };

  // All subscriptions in this direction use this node's default mutually exclusive group.
  subscriptions_.push_back(create_subscription<MsgType>(addIfaceNS(_sub_topic), qos, callback));
}
}  // namespace tobas

int main(int _argc, char* _argv[])
{
  rclcpp::init(_argc, _argv);
  const auto node = std::make_shared<tobas::TopicInterfaceIfaceToLogicNode>();
  rclcpp::executors::EventsCBGExecutor exec(rclcpp::ExecutorOptions(), 1);
  exec.add_node(node);
  exec.spin();
}
