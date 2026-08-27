// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

#include <rclcpp/node.hpp>

namespace tobas
{
namespace gui
{
namespace sim
{
bool waitUntilGazeboServerReady();
bool waitUntilGazeboRenderingReady();
bool waitUntilGazeboShutdown(rclcpp::Node::SharedPtr node, std::chrono::milliseconds timeout);
void killGazeboServer();
}  // namespace sim
}  // namespace gui
}  // namespace tobas
