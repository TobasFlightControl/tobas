// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

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

  std::expected<void, std::string> validate() const override;

  std::expected<void, std::string> load(const YAML::Node& _node) override;
  YAML::Node dump() const override;

  PropulsionSystem type() const override;

  double minSpeed(const std::string& _link_name) override;
  double maxSpeed(const std::string& _link_name) override;

  double minThrust(const std::string& _link_name) override;
  double maxThrust(const std::string& _link_name) override;

  double thrustFromThrottle(const std::string& _link_name, double _throttle) override;

  ElectricRotorConfig::SharedPtr getRotor(const std::string& _link_name);
  ElectricRotorConfig::ConstSharedPtr getRotor(const std::string& _link_name) const;
};
}  // namespace tobas
