// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>

#include "../../hardware_interface.hpp"

namespace tobas
{
class EngineConfig
{
public:
  std::pair<double, double> engine_const = { 0.0, 0.0 };  ///< A, B (memo: 3-28)
  HardwareInterface hw_iface = HardwareInterface::kOther;

  std::expected<void, std::string> validate() const;

  std::expected<void, std::string> load(const YAML::Node& _node);
  YAML::Node dump() const;

  /** Compute engine torque [Nm] from speed [rad/s] and throttle [0, 1]. */
  double computeTorque(double _speed, double _throttle);

  /** Compute engine throttle [0, 1] from torque [N] and speed [rad/s]. */
  double computeThrottle(double _torque, double _speed);

  friend std::ostream& operator<<(std::ostream& _os, const EngineConfig& _arg);
};
}  // namespace tobas
