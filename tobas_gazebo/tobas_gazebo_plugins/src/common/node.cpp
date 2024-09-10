
#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>

#include "../../include/tobas_gazebo_plugins/common/node.hpp"

using namespace std;

namespace gazebo
{
BaseNode::BaseNode()
{
}

void BaseNode::initialize(const string& name, const sdf::ElementConstPtr& sdf)
{
  ignmsg << "Initializing \"" << name << "\"." << endl;

  name_ = name;

  if (!sdf->Get<string>("robotNamespace", ns_, "/"))
    gzwarn << "[" << name << "] Namespace is not specified." << endl;

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  node_ = rclcpp::Node::make_shared(name, ns_);
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
