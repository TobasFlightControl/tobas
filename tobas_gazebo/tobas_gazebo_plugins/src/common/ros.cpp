#include <tobas_constants/constants.hpp>

#include "../../include/tobas_gazebo_plugins/common/ros.hpp"
#include "../../include/tobas_gazebo_plugins/common/sdf.hpp"

using namespace std;

namespace gazebo
{
BaseNode::BaseNode(const string& name) : name_(name)
{
}

void BaseNode::initialize(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  node_ = rclcpp::Node::make_shared(name_, ns);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  auto spin = [this]() { executor_->spin(); };
  spin_thread_ = thread(spin);

  message_pub_ = createPublisher<tobas_std_msgs::msg::Message>(tobas::kMessageTopic);

  TOBAS_INFO("Initializing \"", name_, "\".");
}

const string& BaseNode::name() const
{
  return name_;
}

const std::string& BaseNode::ns() const
{
  return ns_;
}

void BaseNode::rclcppLog(uint8_t level, const string& text) const
{
  switch (level)
  {
    case tobas_std_msgs::msg::Message::LEVEL_DEBUG:
      RCLCPP_DEBUG_STREAM(node_->get_logger(), text);
      break;
    case tobas_std_msgs::msg::Message::LEVEL_INFO:
      RCLCPP_INFO_STREAM(node_->get_logger(), text);
      break;
    case tobas_std_msgs::msg::Message::LEVEL_WARN:
      RCLCPP_WARN_STREAM(node_->get_logger(), text);
      break;
    case tobas_std_msgs::msg::Message::LEVEL_ERROR:
      RCLCPP_ERROR_STREAM(node_->get_logger(), text);
      break;
    case tobas_std_msgs::msg::Message::LEVEL_FATAL:
      RCLCPP_FATAL_STREAM(node_->get_logger(), text);
      break;
    default:
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Invalid log level: " << static_cast<int>(level));
      break;
  }
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

string BaseNode::createID(const char* file, int line)
{
  return string(file) + ":" + to_string(line);
}
}  // namespace gazebo
