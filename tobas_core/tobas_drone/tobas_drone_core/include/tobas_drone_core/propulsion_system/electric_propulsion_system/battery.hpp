// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
class BatteryConfig
{
public:
  double nominal_voltage = 0.0;      // Nominal voltage [V].
  double max_voltage = 0.0;          // Voltage at full charge [V].
  double sag_voltage = 0.0;          // Voltage where discharge characteristics change abruptly [V].
  double max_current = 0.0;          // Maximum continuous current [A].
  double internal_resistance = 0.0;  // Internal resistance [Ω].

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;
};
}  // namespace tobas
