#pragma once

#include <rclcpp/logger.hpp>

namespace tobas
{
/**
 * @brief Call once after creating a node to initialize logging namespaces.
 */
void setNodeLoggerName(const std::string& name);

/**
 * @brief Creates a namespaced logger.
 */
rclcpp::Logger getLogger(const std::string& name);
}  // namespace tobas
