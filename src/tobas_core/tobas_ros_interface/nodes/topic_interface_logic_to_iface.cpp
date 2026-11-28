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
#include <tobas_real_common/ros_interface.hpp>
#include <tobas_tools/util.hpp>

#include <tobas_drone_msgs/msg/drone.hpp>
#include <tobas_kdl_msgs/msg/tree.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/engine_state.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/gnss.hpp>
#include <tobas_msgs/msg/heartbeat.hpp>
#include <tobas_msgs/msg/imu.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/magnetic_field.hpp>
#include <tobas_msgs/msg/message.hpp>
#include <tobas_msgs/msg/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/rosbag_state.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/sbus.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>

namespace tobas
{
class TopicInterfaceLogicToIfaceNode : public BaseNode
{
  using super = BaseNode;

public:
  explicit TopicInterfaceLogicToIfaceNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

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

TopicInterfaceLogicToIfaceNode::TopicInterfaceLogicToIfaceNode(const rclcpp::NodeOptions& _options)
  : super("topic_interface_logic_to_iface", nodeOptions_Default(_options))
{
  addTopic<tobas_msgs::msg::Message>(topic::kMessage, topic::kMessage, false, true, 100);
  addTopic<tobas_drone_msgs::msg::Drone>(topic::kDrone, topic::kDrone, true, true);
  addTopic<tobas_kdl_msgs::msg::Tree>(topic::kKdlTree, topic::kKdlTree, true, true);
  addTopic<tobas_msgs::msg::Battery>(addThrotNS(topic::kBattery), topic::kBattery);
  addTopic<tobas_msgs::msg::EngineState>(addThrotNS(topic::kEngineState), topic::kEngineState);
  addTopic<tobas_msgs::msg::Cpu>(topic::kCpu, topic::kCpu);
  addTopic<tobas_msgs::msg::Sbus>(addThrotNS(topic::kSbus), topic::kSbus);
  addTopic<tobas_msgs::msg::RCInput>(addThrotNS(topic::kRcInput), topic::kRcInput);
  addTopic<tobas_msgs::msg::Imu>(addThrotNS(topic::kImuFilt), topic::kImuFilt);
  addTopic<tobas_msgs::msg::MagneticField>(addThrotNS(topic::kMagneticField), topic::kMagneticField);
  addTopic<tobas_msgs::msg::FluidPressure>(addThrotNS(topic::kAirPressure), topic::kAirPressure);
  addTopic<tobas_msgs::msg::Gnss>(topic::kGnss, topic::kGnss);
  addTopic<tobas_msgs::msg::RotorStateArray>(addThrotNS(topic::kRotorStates), topic::kRotorStates);
  addTopic<tobas_msgs::msg::RotorLivelinessArray>(topic::kRotorLiv, topic::kRotorLiv);
  addTopic<tobas_msgs::msg::JointStateArray>(addThrotNS(topic::kJointStates), topic::kJointStates);
  addTopic<tobas_msgs::msg::OdometryWithCovarianceStamped>(addThrotNS(topic::kOdometry), topic::kOdometry);
  addTopic<tobas_msgs::msg::Arming>(topic::kArming, topic::kArming);
  addTopic<tobas_msgs::msg::VehicleHealth>(topic::kVehicleHealth, topic::kVehicleHealth);
  addTopic<tobas_msgs::msg::Imu>(addThrotNS(real::topic::kImuRaw), real::topic::kImuRaw);
  addTopic<tobas_msgs::msg::MagneticField>(addThrotNS(real::topic::kMagneticField), real::topic::kMagneticField);
  addTopic<tobas_msgs::msg::RosbagState>(topic::kRosbagState, topic::kRosbagState);
  addTopic<tobas_msgs::msg::Heartbeat>(topic::kHeartbeat, topic::kHeartbeat);
}

template <typename MsgType>
void TopicInterfaceLogicToIfaceNode::addTopic(
  const std::string& _sub_topic,
  const std::string& _pub_topic,
  bool _latch,
  bool _reliable,
  size_t _queue_size)
{
  const ros2::qos::QoS qos(_latch, _reliable, _queue_size);
  const auto publisher = create_publisher<MsgType>(addIfaceNS(_pub_topic), qos);

  const auto callback = [publisher](const typename MsgType::ConstSharedPtr& _msg)
  {
    // The standalone default disables intra-process delivery, avoiding an extra rclcpp copy here.
    publisher->publish(*_msg);
  };

  // All subscriptions in this direction use this node's default mutually exclusive group.
  subscriptions_.push_back(create_subscription<MsgType>(_sub_topic, qos, callback));
}
}  // namespace tobas

int main(int _argc, char* _argv[])
{
  rclcpp::init(_argc, _argv);
  const auto node = std::make_shared<tobas::TopicInterfaceLogicToIfaceNode>();
  rclcpp::executors::EventsCBGExecutor exec(rclcpp::ExecutorOptions(), 1);
  exec.add_node(node);
  exec.spin();
}
