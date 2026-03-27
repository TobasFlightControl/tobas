#pragma once

#include <QObject>
#include <rclcpp/rclcpp.hpp>

#include "./declare.hpp"

namespace tobas
{
namespace gui
{
class RosQtBridge : public QObject
{
  Q_OBJECT

  using self = RosQtBridge;
  using super = QObject;

Q_SIGNALS:
  void messageReceived(const tobas_msgs::msg::Message::ConstSharedPtr& msg);
  void batteryReceived(const tobas_msgs::msg::Battery::ConstSharedPtr& msg);
  void engineStateReceived(const tobas_msgs::msg::EngineState::ConstSharedPtr& msg);
  void cpuReceived(const tobas_msgs::msg::Cpu::ConstSharedPtr& msg);
  void sbusReceived(const tobas_msgs::msg::Sbus::ConstSharedPtr& msg);
  void rcInputReceived(const tobas_msgs::RCInput::ConstSharedPtr& msg);
  void imuReceived(const tobas_msgs::Imu::ConstSharedPtr& msg);
  void magReceived(const tobas_msgs::MagneticField::ConstSharedPtr& msg);
  void airPressureReceived(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& msg);
  void gnssReceived(const tobas_msgs::Gnss::ConstSharedPtr& msg);
  void rotorStatesReceived(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& msg);
  void rotorLivelinessReceived(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& msg);
  void jointStatesReceived(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& msg);
  void odomReceived(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& msg);
  void armingReceived(const tobas_msgs::msg::Arming::ConstSharedPtr& msg);
  void vehicleHealthReceived(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& msg);
  void rosbagStateReceived(const tobas_msgs::msg::RosbagState::ConstSharedPtr& msg);
  void localHeartbeatReceived(const tobas_msgs::msg::Heartbeat::ConstSharedPtr& msg);
  void remoteHeartbeatReceived(const tobas_msgs::msg::Heartbeat::ConstSharedPtr& msg);
  void rawImuReceived(const tobas_msgs::Imu::ConstSharedPtr& msg);
  void rawMagReceived(const tobas_msgs::MagneticField::ConstSharedPtr& msg);

public:
  explicit RosQtBridge(rclcpp::Node::SharedPtr node);

  void initializeScopedTopics(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> global_subs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> scoped_subs_;

  template <typename MsgType, auto SignalType>
  void add(const std::string& topic, std::vector<rclcpp::SubscriptionBase::SharedPtr>& buf);

  template <typename MsgType, auto SignalType>
  void addGlobal(const std::string& topic);

  template <typename MsgType, auto SignalType>
  void addScoped(const std::string& ns, const std::string& topic);
};
}  // namespace gui
}  // namespace tobas
