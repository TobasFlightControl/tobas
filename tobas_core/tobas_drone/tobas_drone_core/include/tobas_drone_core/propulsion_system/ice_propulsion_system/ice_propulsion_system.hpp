// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "../propulsion_system.hpp"
#include "./engine.hpp"
#include "./ice_rotor.hpp"

namespace tobas
{
class IcePropulsionSystemConfig : public PropulsionSystemConfig
{
  using self = IcePropulsionSystemConfig;

public:
  using SharedPtr = std::shared_ptr<IcePropulsionSystemConfig>;
  using ConstSharedPtr = std::shared_ptr<const IcePropulsionSystemConfig>;

  EngineConfig engine;

  bool isValid() const override;

  bool load(const YAML::Node& node) override;
  YAML::Node dump() const override;

  PropulsionSystem type() const override;

  double minSpeed(const std::string& link_name) override;
  double maxSpeed(const std::string& link_name) override;

  double minThrust(const std::string& link_name) override;
  double maxThrust(const std::string& link_name) override;

  double thrustFromThrottle(const std::string& link_name, double throttle) override;

  IceRotorConfig::SharedPtr getRotor(const std::string& link_name);
  IceRotorConfig::ConstSharedPtr getRotor(const std::string& link_name) const;

private:
  std::optional<double> max_engine_speed_;

  /* Maximum engine speed when the average propeller pitch angle is fixed [rad/s]. */
  double maxEngineSpeed();

  /* Compute steady-state speed from engine throttle with the average propeller pitch angle fixed (memo: 3-29). */
  double computeEngineSpeed(double throttle) const;

  /* Function passed to the Newton-method solver (memo: 3-29). */
  double speedFunc(double throttle, double omega) const;
  double speedFuncDeriv(double throttle, double omega) const;

  double calc_phi(double throttle) const;
  double calc_f(double throttle) const;
  double calc_k() const;
};
}  // namespace tobas
