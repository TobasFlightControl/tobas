#include "tobas_rqt_bridge/bridge.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_ros2_tools/register.hpp>

namespace gui
{
RosQtBridge::RosQtBridge(rclcpp::Node::SharedPtr node) : node_(node)
{
}

void RosQtBridge::initialize(const std::string& ns)
{
  subscriptions_.clear();

  add<tobas_msgs::msg::Battery, &self::batteryReceived>(ns, tobas::kBatteryTopic);
  add<tobas_msgs::msg::EngineState, &self::engineReceived>(ns, tobas::kEngineStateTopic);
  add<tobas_msgs::msg::Cpu, &self::cpuReceived>(ns, tobas::kCpuTopic);
  add<tobas_msgs::msg::Sbus, &self::sbusReceived>(ns, tobas::kSbusTopic);
  add<tobas_msgs::RCInput, &self::rcInputReceived>(ns, tobas::kRcInputTopic);
  add<tobas_msgs::Gnss, &self::gnssReceived>(ns, tobas::kGnssTopic);
  add<tobas_msgs::msg::RotorStateArray, &self::rotorStatesReceived>(ns, tobas::kRotorStatesTopic);
  add<tobas_msgs::msg::RotorLivelinessArray, &self::rotorLivelinessesReceived>(ns, tobas::kRotorLivelinessesTopic);
  add<tobas_msgs::msg::JointStateArray, &self::jointStatesReceived>(ns, tobas::kJointStatesTopic);
  add<tobas_msgs::Odometry, &self::odomReceived>(ns, tobas::kOdometryTopic);
  add<tobas_msgs::msg::Arming, &self::armingReceived>(ns, tobas::kArmingTopic);
  add<tobas_msgs::msg::PreArmCheck, &self::preArmCheckReceived>(ns, tobas::kPreArmCheckTopic);
  add<tobas_msgs::msg::PostArmCheck, &self::postArmCheckReceived>(ns, tobas::kPostArmCheckTopic);
  add<tobas_msgs::ImuStamped, &self::rawImuReceived>(ns, real::kImuTopic);
  add<tobas_msgs::MagneticFieldStamped, &self::rawMagReceived>(ns, real::kMagTopic);
  add<tobas_msgs::msg::RosbagState, &self::rosbagStateReceived>(ns, tobas::kRosbagStateTopic);
}

template <typename MsgType, auto SignalType>
void RosQtBridge::add(const std::string& ns, const std::string& topic, bool latch, bool reliable, size_t queue_size)
{
  const auto qos = ros2::makeQoS(latch, reliable, queue_size);
  const auto cb = [this](const typename MsgType::ConstSharedPtr& msg) { (this->*SignalType)(msg); };
  const auto remote_topic = path::join("/", ns, tobas::kRemoteIfaceTopicNS, topic);
  subscriptions_[topic] = node_->create_subscription<MsgType>(remote_topic, qos, cb);
}
}  // namespace gui
