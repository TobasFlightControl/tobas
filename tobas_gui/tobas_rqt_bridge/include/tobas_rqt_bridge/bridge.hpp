#pragma once

#include <QObject>

#include <tobas_ros2_tools/definitions.hpp>
#include <tobas_ros2_tools/qos.hpp>

#include "./declare.hpp"

namespace gui
{
class RosQtBridge : public QObject
{
  Q_OBJECT

  using self = RosQtBridge;
  using super = QObject;

Q_SIGNALS:
  void batteryReceived(const tobas_msgs::msg::Battery::ConstSharedPtr& msg);
  void engineReceived(const tobas_msgs::msg::EngineState::ConstSharedPtr& msg);
  void cpuReceived(const tobas_msgs::msg::Cpu::ConstSharedPtr& msg);
  void sbusReceived(const tobas_msgs::msg::Sbus::ConstSharedPtr& msg);
  void rcInputReceived(const tobas_msgs::RCInput::ConstSharedPtr& msg);
  void gnssReceived(const tobas_msgs::Gnss::ConstSharedPtr& msg);
  void rotorStatesReceived(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& msg);
  void rotorLivelinessesReceived(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& msg);
  void jointStatesReceived(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& msg);
  void odomReceived(const tobas_msgs::Odometry::ConstSharedPtr& msg);
  void armingReceived(const tobas_msgs::msg::Arming::ConstSharedPtr& msg);
  void preArmCheckReceived(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& msg);
  void postArmCheckReceived(const tobas_msgs::msg::PostArmCheck::ConstSharedPtr& msg);
  void rawImuReceived(const tobas_msgs::ImuStamped::ConstSharedPtr& msg);
  void rawMagReceived(const tobas_msgs::MagneticFieldStamped::ConstSharedPtr& msg);
  void rosbagStateReceived(const tobas_msgs::msg::RosbagState::ConstSharedPtr& msg);

public:
  explicit RosQtBridge(rclcpp::Node::SharedPtr node);

  void initialize(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  template <typename MsgType, auto SignalType>
  void add(
    const std::string& ns,
    const std::string& topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);
};
}  // namespace gui
