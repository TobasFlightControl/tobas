// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <QString>
#include <rclcpp/node.hpp>

#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/tree.hpp>

namespace tobas
{
namespace gui
{
namespace gcs
{
struct VehicleConfiguration
{
  kdl::Tree tree;
  Drone drone;
};

/** Receive and validate both latched model topics without blocking the GUI event loop. */
std::expected<VehicleConfiguration, QString>
loadVehicleConfiguration(const rclcpp::Node::SharedPtr& node, const std::string& ns, int timeout_ms = 10000);
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
