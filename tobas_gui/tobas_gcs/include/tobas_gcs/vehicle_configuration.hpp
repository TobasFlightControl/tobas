// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <QString>
#include <rclcpp/node.hpp>

#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/tree.hpp>

namespace tobas::gui::gcs
{
struct VehicleConfiguration
{
  kdl::Tree tree;
  Drone drone;
};

/** Receive and validate both latched model topics without blocking the GUI event loop. */
std::expected<VehicleConfiguration, QString>
loadVehicleConfiguration(const rclcpp::Node::SharedPtr& _node, const std::string& _ns, int _timeout_ms = 10000);
}  // namespace tobas::gui::gcs
