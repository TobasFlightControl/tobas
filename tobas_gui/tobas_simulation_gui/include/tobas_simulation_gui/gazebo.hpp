#pragma once

#include <sys/types.h>

#include <expected>

#include <QString>
#include <rclcpp/node.hpp>

namespace tobas
{
namespace gui
{
namespace sim
{
bool waitUntilGazeboServerReady();
bool waitUntilGazeboRenderingReady();

std::expected<void, QString> killGazebo(rclcpp::Node::SharedPtr node, pid_t pid);
}  // namespace sim
}  // namespace gui
}  // namespace tobas
