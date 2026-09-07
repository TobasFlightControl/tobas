// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rosbag2_cpp/writer.hpp>

#include <tobas_constants/path.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_path_tools/core.hpp>
#include <tobas_ros2_tools/util.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <tobas_debug_msgs/msg/multicopter_controller_feedback.hpp>
#include <tobas_debug_msgs/msg/observer_feedback.hpp>
#include <tobas_drone_msgs/msg/drone.hpp>
#include <tobas_kdl_msgs/msg/tree.hpp>
#include <tobas_kdl_msgs/msg/wrench_stamped.hpp>
#include <tobas_msgs/msg/gnss.hpp>
#include <tobas_msgs/msg/imu.hpp>
#include <tobas_msgs/msg/magnetic_field.hpp>
#include <tobas_msgs/msg/odometry_stamped.hpp>
#include <tobas_msgs/msg/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/rosbag_state.hpp>
#include <tobas_msgs/msg/vibration_level.hpp>
#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>

namespace fs = std::filesystem;

namespace tobas
{
class RosbagRecorderNode : public BaseNode
{
  using self = RosbagRecorderNode;
  using super = BaseNode;

  using StartSrv = tobas_msgs::srv::BagRecordStart;
  using StopSrv = tobas_msgs::srv::BagRecordStop;
  using CleanSrv = std_srvs::srv::Trigger;

public:
  explicit RosbagRecorderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  const std::string ns_;
  const fs::path rosbag_dir_;

  rosbag2_cpp::Writer writer_;
  fs::path file_path_;
  bool recording_ = false;
  rclcpp::Time start_time_;
  size_t msg_cnt_;

  // ROS standard message buffers
  tobas_drone_msgs::msg::Drone drone_;
  tobas_kdl_msgs::msg::Tree tree_;
  tobas_msgs::msg::RCInput rcin_;
  tobas_msgs::msg::Imu imu_;
  tobas_msgs::msg::MagneticField mag_;
  tobas_msgs::msg::Gnss gnss_;
  tobas_msgs::msg::OdometryStamped odom_;
  tobas_msgs::msg::OdometryWithCovarianceStamped odom_cov_;
  tobas_msgs::msg::VibrationLevel vibe_;
  tobas_kdl_msgs::msg::WrenchStamped dist_force_;
  tobas_debug_msgs::msg::ObserverFeedback obsv_fb_;
  tobas_debug_msgs::msg::MulticopterControllerFeedback mr_ctrl_fb_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RosbagState> rosbag_state_pub_;

  // Subscribers
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

  // Service servers
  ros2::ServiceServerPtr<StartSrv> start_srv_;
  ros2::ServiceServerPtr<StopSrv> stop_srv_;
  ros2::ServiceServerPtr<CleanSrv> clean_srv_;

  // Timers
  ros2::TimerPtr main_timer_;

  void registerModelSubscribers();
  void registerSensorSubscribers();
  void registerStateSubscribers();
  void registerCommandSubscribers();
  void registerDebugSubscribers();

  /* Get the available disk space in bytes. */
  size_t getDiskAvailableSize() const noexcept;

  void publishRosbagState();

  template <typename MsgType>
  inline void write(const MsgType& msg, const char* topic) noexcept;

  template <typename MsgType>
  void addStandardMsgSub(
    const char* topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename ExtMsgType, typename RawMsgType>
  void addTypeAdaptedMsgSub(
    RawMsgType& raw_msg,
    const char* topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename MsgType>
  void standardMsgCb(const typename MsgType::ConstSharedPtr& msg, const char* topic);

  template <typename ExtMsgType, typename RawMsgType>
  void typeAdaptedMsgCb(const typename ExtMsgType::ConstSharedPtr& ext_msg, RawMsgType& raw_msg, const char* topic);

  void startCb(const StartSrv::Request::ConstSharedPtr& req, const StartSrv::Response::SharedPtr& res);
  void stopCb(const StopSrv::Request::ConstSharedPtr& req, const StopSrv::Response::SharedPtr& res);
  void cleanCb(const CleanSrv::Request::ConstSharedPtr& req, const CleanSrv::Response::SharedPtr& res);

  void mainTimerCb();
};

template <typename MsgType>
inline void RosbagRecorderNode::write(const MsgType& msg, const char* topic) noexcept
{
  try {
    writer_.write(msg, ns_ + topic, now());
  }
  catch (const std::exception& e) {
    TOBAS_ERROR("Failed to write \"", topic, "\": ", e.what());
    return;
  }

  ++msg_cnt_;
}

template <typename MsgType>
void RosbagRecorderNode::addStandardMsgSub(const char* topic, bool latch, bool reliable, size_t queue_size)
{
  const ros2::qos::QoS qos(latch, reliable, queue_size);
  const auto cb = [this, topic](const typename MsgType::ConstSharedPtr& msg) { standardMsgCb<MsgType>(msg, topic); };
  const auto sub = create_subscription<MsgType>(topic, qos, cb);
  subs_.push_back(sub);
}

template <typename ExtMsgType, typename RawMsgType>
void RosbagRecorderNode::addTypeAdaptedMsgSub(
  RawMsgType& raw_msg,
  const char* topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  const ros2::qos::QoS qos(latch, reliable, queue_size);
  const auto cb = [this, &raw_msg, topic](const typename ExtMsgType::ConstSharedPtr& ext_msg)
  { typeAdaptedMsgCb<ExtMsgType, RawMsgType>(ext_msg, raw_msg, topic); };
  const auto sub = create_subscription<ExtMsgType>(topic, qos, cb);
  subs_.push_back(sub);
}

template <typename MsgType>
void RosbagRecorderNode::standardMsgCb(const typename MsgType::ConstSharedPtr& msg, const char* topic)
{
  if (!recording_) {
    return;
  }

  write(*msg, topic);
}

template <typename ExtMsgType, typename RawMsgType>
void RosbagRecorderNode::typeAdaptedMsgCb(
  const typename ExtMsgType::ConstSharedPtr& ext_msg,
  RawMsgType& raw_msg,
  const char* topic)
{
  if (!recording_) {
    return;
  }

  // Subscribe through TypeAdapter and convert to a ROS message here to prevent serialization on the publisher side.
  rclcpp::TypeAdapter<ExtMsgType, RawMsgType>::convert_to_ros_message(*ext_msg, raw_msg);

  write(raw_msg, topic);
}
}  // namespace tobas
