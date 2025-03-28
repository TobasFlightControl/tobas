#pragma once

#include <rclcpp/logger.hpp>
#include <string>

namespace tobas
{
// @brief Call once after creating a node to initialize logging namespaces.
// @code{C++}
// rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_group");
// setNodeLoggerName(node->get_name());
// @endcode
void setNodeLoggerName(const std::string& name);

// @brief Creates a namespaced logger.
rclcpp::Logger getLogger(const std::string& name);
}  // namespace tobas
