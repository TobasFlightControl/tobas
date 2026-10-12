// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/ice_rotor.hpp"

#include <tobas_yaml_tools/convert/range.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
namespace
{
constexpr char kGearRatioKey[] = "gear_ratio";
constexpr char kPitchLimitKey[] = "pitch_limit";
constexpr char kCenterPitchKey[] = "center_pitch";
constexpr char kMotorConstKey[] = "motor_constant";
constexpr char kMomentConstKey[] = "moment_constant";
constexpr char kHardwareIfaceKey[] = "hw_iface";
}  // namespace

std::expected<void, std::string> IceRotorConfig::validate() const
{
  if (const auto result = super::validate(); !result) {
    return result;
  }

  if (gear_ratio <= 0.0) {
    return std::unexpected("Gear ratio must be positive.");
  }

  if (!pitch_limit.isValid()) {
    return std::unexpected("Pitch angle limit is invalid.");
  }

  if (!pitch_limit.inRange(center_pitch)) {
    return std::unexpected("Center pitch is out of its limit.");
  }

  if (const auto result = motor_const.validate(); !result) {
    return result;
  }

  if (const auto result = moment_const.validate(); !result) {
    return result;
  }

  return {};
}

std::expected<void, std::string> IceRotorConfig::load(const YAML::Node& node)
{
  if (!node.IsDefined() || !node.IsMap()) {
    return std::unexpected("Configuration node must be a map.");
  }

  if (const auto result = super::load(node); !result) {
    return result;
  }

  if (const auto result = yaml::load(kGearRatioKey, node, gear_ratio); !result) {
    return result;
  }

  if (const auto result = yaml::load(kPitchLimitKey, node, pitch_limit); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCenterPitchKey, node, center_pitch); !result) {
    return result;
  }

  if (const auto result = motor_const.load(node[kMotorConstKey]); !result) {
    return std::unexpected("Motor constant: " + result.error());
  }

  if (const auto result = moment_const.load(node[kMomentConstKey]); !result) {
    return std::unexpected("Moment constant: " + result.error());
  }

  if (const auto result = yaml::load(kHardwareIfaceKey, node, hw_iface); !result) {
    return result;
  }

  return {};
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
