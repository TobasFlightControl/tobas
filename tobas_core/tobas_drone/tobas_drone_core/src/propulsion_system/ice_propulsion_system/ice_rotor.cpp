// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/ice_rotor.hpp"

#include <tobas_yaml_tools/convert/range.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
bool IceRotorConfig::isValid() const
{
  if (!super::isValid()) {
    return false;
  }

  if (gear_ratio <= 0.) {
    std::cerr << "Gear ratio must be positive." << std::endl;
    return false;
  }

  if (!pitch_limit.isValid()) {
    std::cerr << "Pitch angle limit is invalid." << std::endl;
    return false;
  }

  if (!pitch_limit.inRange(center_pitch)) {
    std::cerr << "Center pitch is out of its limit." << std::endl;
    return false;
  }

  if (!motor_const.isValid()) {
    return false;
  }

  if (!moment_const.isValid()) {
    return false;
  }

  return true;
}

bool IceRotorConfig::load(const YAML::Node& node)
{
  if (!super::load(node)) {
    return false;
  }

  if (!yaml::load(kGearRatioKey, node, gear_ratio)) {
    return false;
  }

  if (!yaml::load(kPitchLimitKey, node, pitch_limit)) {
    return false;
  }

  if (!yaml::load(kCenterPitchKey, node, center_pitch)) {
    return false;
  }

  if (!motor_const.load(node[kMotorConstKey])) {
    return false;
  }

  if (!moment_const.load(node[kMomentConstKey])) {
    return false;
  }

  if (!yaml::load(kHardwareIfaceKey, node, hw_iface)) {
    return false;
  }

  return true;
}

YAML::Node IceRotorConfig::dump() const
{
  auto node = super::dump();

  node[kGearRatioKey] = yaml::format(gear_ratio);
  node[kPitchLimitKey] = pitch_limit;
  node[kCenterPitchKey] = yaml::format(center_pitch);
  node[kMotorConstKey] = motor_const.dump();
  node[kMomentConstKey] = moment_const.dump();
  node[kHardwareIfaceKey] = hw_iface;

  return node;
}
}  // namespace tobas
