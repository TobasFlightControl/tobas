
#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>

#include "../../include/tobas_gazebo_plugins/common/node.hpp"

using namespace std;

namespace gazebo
{
BaseNode::BaseNode(const string& name) : name_(name)
{
}

void BaseNode::initialize(const sdf::ElementConstPtr& sdf)
{
  ignmsg << "Initializing \"" << name_ << "\"." << endl;

  if (!sdf->Get<string>("robotNamespace", ns_, "/"))
    gzwarn << "[" << name_ << "] Namespace is not specified." << endl;

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  node_ = rclcpp::Node::make_shared(name_, ns_);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  auto spin = [this]() { executor_->spin(); };
  spin_thread_ = thread(spin);

  message_pub_ = createPublisher<tobas_std_msgs::msg::Message>(path::join(ns_, tobas::kMessageTopic));
}

const string& BaseNode::name() const
{
  return name_;
}

const std::string& BaseNode::ns() const
{
  return ns_;
}

rclcpp::QoS BaseNode::makeQoS(bool latch, bool reliable, size_t queue_size)
{
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, queue_size));

  if (latch)
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  else
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  if (reliable)
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  else
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  return qos;
}

void BaseNode::gazeboLog(uint8_t level, const string& text) const
{
  switch (level)
  {
    case tobas_std_msgs::msg::Message::LEVEL_DEBUG:
      tbsdbg << text << endl;
      break;
    case tobas_std_msgs::msg::Message::LEVEL_INFO:
      tbsmsg << text << endl;
      break;
    case tobas_std_msgs::msg::Message::LEVEL_WARN:
      tbswarn << text << endl;
      break;
    case tobas_std_msgs::msg::Message::LEVEL_ERROR:
      tbserr << text << endl;
      break;
    case tobas_std_msgs::msg::Message::LEVEL_FATAL:
      tbserr << text << endl;
      break;
    default:
      tbserr << "Invalid log level: " << static_cast<int>(level) << endl;
      break;
  }
}

string BaseNode::createID(const char* file, int line)
{
  return string(file) + ":" + to_string(line);
}
}  // namespace gazebo
