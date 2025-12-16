#include "tobas_gazebo_system_plugins/common/node.hpp"

#include <tobas_constants/constants.hpp>

namespace gazebo
{
BaseNode::BaseNode()
{
}

BaseNode::~BaseNode()
{
  if (executor_) {
    executor_->cancel();
    spin_thread_.join();
  }
}

void BaseNode::initialize(const std::string& name, const sdf::ElementConstPtr& sdf)
{
  gzmsg << "Initializing \"" << name << "\"." << std::endl;

  name_ = name;

  if (!sdf->Get<std::string>("robotNamespace", ns_, "/")) {
    gzwarn << "[" << name << "] Namespace is not specified." << std::endl;
  }

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  node_ = rclcpp::Node::make_shared(name, ns_);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  const auto spin = [this]() { executor_->spin(); };
  spin_thread_ = std::thread(spin);

  message_pub_ = createPublisher<tobas_msgs::msg::Message>(tobas::kMessageTopic);
}

const std::string& BaseNode::name() const
{
  return name_;
}

const std::string& BaseNode::ns() const
{
  return ns_;
}

void BaseNode::gazeboLog(uint8_t level, const std::string& text) const
{
  switch (level) {
    case tobas_msgs::msg::Message::LEVEL_DEBUG:
      tbsdbg << text << std::endl;
      break;
    case tobas_msgs::msg::Message::LEVEL_INFO:
      tbsmsg << text << std::endl;
      break;
    case tobas_msgs::msg::Message::LEVEL_WARN:
      tbswarn << text << std::endl;
      break;
    case tobas_msgs::msg::Message::LEVEL_ERROR:
      tbserr << text << std::endl;
      break;
    case tobas_msgs::msg::Message::LEVEL_FATAL:
      tbserr << text << std::endl;
      break;
    default:
      tbserr << "Invalid log level: " << static_cast<int>(level) << std::endl;
      break;
  }
}

std::string BaseNode::createID(const char* file, int line)
{
  return std::string(file) + ":" + std::to_string(line);
}
}  // namespace gazebo
