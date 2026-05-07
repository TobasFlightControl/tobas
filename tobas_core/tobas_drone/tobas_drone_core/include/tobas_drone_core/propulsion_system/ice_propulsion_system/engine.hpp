// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <iostream>

#include <yaml-cpp/yaml.h>

#include "../../hardware_interface.hpp"

namespace tobas
{
class EngineConfig
{
  static constexpr char kEngineConstantKey[] = "engine_constant";
  static constexpr char kHardwareIfaceKey[] = "hw_iface";

public:
  std::pair<double, double> engine_const = { 0., 0. };  // A, B (memo: 3-28)
  HardwareInterface hw_iface = HardwareInterface::kOther;

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  /* Compute engine torque [Nm] from speed [rad/s] and throttle [0, 1]. */
  double computeTorque(double speed, double throttle);

  /* Compute engine throttle [0, 1] from torque [N] and speed [rad/s]. */
  double computeThrottle(double torque, double speed);

  friend std::ostream& operator<<(std::ostream& os, const EngineConfig& arg);
};
}  // namespace tobas
