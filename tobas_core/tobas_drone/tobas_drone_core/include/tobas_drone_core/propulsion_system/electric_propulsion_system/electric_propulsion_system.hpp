// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "../propulsion_system.hpp"
#include "./battery.hpp"
#include "./electric_rotor.hpp"

namespace tobas
{
class ElectricPropulsionSystemConfig : public PropulsionSystemConfig
{
public:
  using SharedPtr = std::shared_ptr<ElectricPropulsionSystemConfig>;
  using ConstSharedPtr = std::shared_ptr<const ElectricPropulsionSystemConfig>;

  BatteryConfig battery;

  bool isValid() const override;

  bool load(const YAML::Node& node) override;
  YAML::Node dump() const override;

  PropulsionSystem type() const override;

  double minSpeed(const std::string& link_name) override;
  double maxSpeed(const std::string& link_name) override;

  double minThrust(const std::string& link_name) override;
  double maxThrust(const std::string& link_name) override;

  double thrustFromThrottle(const std::string& link_name, double throttle) override;

  ElectricRotorConfig::SharedPtr getRotor(const std::string& link_name);
  ElectricRotorConfig::ConstSharedPtr getRotor(const std::string& link_name) const;
};
}  // namespace tobas
