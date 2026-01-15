#include "tobas_rqt_bridge/bridge.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_ros2_tools/qos.hpp>

using namespace tobas;

namespace gui
{
RosQtBridge::RosQtBridge(rclcpp::Node::SharedPtr node) : node_(node)
{
}

void RosQtBridge::initializeScopedTopics(const std::string& ns)
{
  static constexpr auto rm = kRemoteIfaceTopicNS;

  scoped_subs_.clear();

  addScoped<tobas_msgs::msg::Message, &self::messageReceived>(ns, path::join(rm, kMessageTopic));
  addScoped<tobas_msgs::msg::Battery, &self::batteryReceived>(ns, path::join(rm, kBatteryTopic));
  addScoped<tobas_msgs::msg::EngineState, &self::engineStateReceived>(ns, path::join(rm, kEngineStateTopic));
  addScoped<tobas_msgs::msg::Cpu, &self::cpuReceived>(ns, path::join(rm, kCpuTopic));
  addScoped<tobas_msgs::msg::Sbus, &self::sbusReceived>(ns, path::join(rm, kSbusTopic));
  addScoped<tobas_msgs::RCInput, &self::rcInputReceived>(ns, path::join(rm, kRcInputTopic));
  addScoped<tobas_msgs::Imu, &self::imuReceived>(ns, path::join(rm, kImuFiltTopic));
  addScoped<tobas_msgs::MagneticField, &self::magReceived>(ns, path::join(rm, kMagTopic));
  addScoped<tobas_msgs::msg::FluidPressure, &self::airPressureReceived>(ns, path::join(rm, kAirPressureTopic));
  addScoped<tobas_msgs::Gnss, &self::gnssReceived>(ns, path::join(rm, kGnssTopic));
  addScoped<tobas_msgs::msg::RotorStateArray, &self::rotorStatesReceived>(ns, path::join(rm, kRotorStatesTopic));
  addScoped<tobas_msgs::msg::RotorLivelinessArray, &self::rotorLivelinessReceived>(ns, path::join(rm, kRotorLivTopic));
  addScoped<tobas_msgs::msg::JointStateArray, &self::jointStatesReceived>(ns, path::join(rm, kJointStatesTopic));
  addScoped<tobas_msgs::Odometry, &self::odomReceived>(ns, path::join(rm, kOdometryTopic));
  addScoped<tobas_msgs::msg::Arming, &self::armingReceived>(ns, path::join(rm, kArmingTopic));
  addScoped<tobas_msgs::msg::VehicleHealth, &self::vehicleHealthReceived>(ns, path::join(rm, kVehicleHealthTopic));
  addScoped<tobas_msgs::msg::RosbagState, &self::rosbagStateReceived>(ns, path::join(rm, kRosbagStateTopic));
  addScoped<tobas_msgs::msg::Heartbeat, &self::remoteHeartbeatReceived>(ns, kHeartbeatTopic);
  addScoped<tobas_msgs::Imu, &self::rawImuReceived>(ns, path::join(rm, real::kImuRawTopic));
  addScoped<tobas_msgs::MagneticField, &self::rawMagReceived>(ns, path::join(rm, real::kMagTopic));
}

template <typename MsgType, auto SignalType>
void RosQtBridge::add(const std::string& topic, std::vector<rclcpp::SubscriptionBase::SharedPtr>& buf)
{
  const auto qos = ros2::makeQoS(false, false, 1);  // 必ず受け取れる設定
  const auto cb = [this](const typename MsgType::ConstSharedPtr& msg) { (this->*SignalType)(msg); };
  buf.push_back(node_->create_subscription<MsgType>(topic, qos, cb));
}

template <typename MsgType, auto SignalType>
void RosQtBridge::addGlobal(const std::string& topic)
{
  add<MsgType, SignalType>(topic, global_subs_);
}

template <typename MsgType, auto SignalType>
void RosQtBridge::addScoped(const std::string& ns, const std::string& topic)
{
  add<MsgType, SignalType>(path::join(ns, topic), scoped_subs_);
}
}  // namespace gui
