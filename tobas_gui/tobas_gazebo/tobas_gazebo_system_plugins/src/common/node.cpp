// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/common/node.hpp"

#include <tobas_constants/ros_interface.hpp>

#define tbsdbg gzdbg << "[" << name_ << "] "
#define tbsmsg gzmsg << "[" << name_ << "] "
#define tbswarn gzwarn << "[" << name_ << "] "
#define tbserr gzerr << "[" << name_ << "] "

namespace tobas
{
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

void BaseNode::initialize(const std::string& _name, const sdf::ElementConstPtr& _sdf)
{
  gzmsg << "Initializing '" << _name << "'.\n";

  name_ = _name;

  if (!_sdf->Get<std::string>("robotNamespace", ns_, "/")) {
    gzwarn << "[" << _name << "] Namespace is not specified.\n";
  }

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.enable_rosout(false);
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.append_parameter_override("start_type_description_service", false);

  node_ = rclcpp::Node::make_shared(_name, ns_, options);

  // The executor constructor creates a guard condition using the ROS context,
  // so defer construction until after `rclcpp::init()` has initialized that context.
  executor_.emplace();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  message_pub_ = createPublisher<tobas_msgs::msg::Message>(topic::kMessage, false, true, 1);
}

const std::string& BaseNode::name() const
{
  return name_;
}

const std::string& BaseNode::ns() const
{
  return ns_;
}

void BaseNode::gazeboLog(uint8_t _level, const std::string& _text) const
{
  switch (_level) {
    case tobas_msgs::msg::Message::LEVEL_DEBUG:
      tbsdbg << _text << '\n';
      break;
    case tobas_msgs::msg::Message::LEVEL_INFO:
      tbsmsg << _text << '\n';
      break;
    case tobas_msgs::msg::Message::LEVEL_WARN:
      tbswarn << _text << '\n';
      break;
    case tobas_msgs::msg::Message::LEVEL_ERROR:
      tbserr << _text << '\n';
      break;
    case tobas_msgs::msg::Message::LEVEL_FATAL:
      tbserr << _text << '\n';
      break;
    default:
      tbserr << "Invalid log level: " << static_cast<int>(_level) << '\n';
      break;
  }
}

std::string BaseNode::createID(const char* _file, int _line)
{
  return std::string(_file) + ":" + std::to_string(_line);
}
}  // namespace gazebo
}  // namespace tobas
