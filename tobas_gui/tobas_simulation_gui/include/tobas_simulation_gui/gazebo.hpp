// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

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

std::expected<void, QString> killGazebo(rclcpp::Node::SharedPtr node);
}  // namespace sim
}  // namespace gui
}  // namespace tobas
