// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/fixed_wing/aerodynamic_coefs.hpp"

#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
namespace
{
constexpr char kCLift0Key[] = "c_lift_0";
constexpr char kCLiftAlphaKey[] = "c_lift_alpha";
constexpr char kCDrag0Key[] = "c_drag_0";
constexpr char kCDragAlphaKey[] = "c_drag_alpha";
constexpr char kCSideBetaKey[] = "c_side_beta";
constexpr char kCRollBetaKey[] = "c_roll_beta";
constexpr char kCRollPKey[] = "c_roll_p";
constexpr char kCRollRKey[] = "c_roll_r";
constexpr char kCPitch0Key[] = "c_pitch_0";
constexpr char kCPitchAlphaKey[] = "c_pitch_alpha";
constexpr char kCPitchAbsBetaKey[] = "c_pitch_abs_beta";
constexpr char kCPitchAlphaRateKey[] = "c_pitch_alpha_rate";
constexpr char kCPitchQKey[] = "c_pitch_q";
constexpr char kCYawBetaKey[] = "c_yaw_beta";
constexpr char kCYawPKey[] = "c_yaw_p";
constexpr char kCYawRKey[] = "c_yaw_r";
}  // namespace

std::expected<void, std::string> AerodynamicCoefficients::validate() const
{
  if (c_lift_0 <= 0) {
    return std::unexpected("c_lift_0 must be positive.");
  }

  if (c_lift_alpha <= 0) {
    return std::unexpected("c_lift_alpha must be positive.");
  }

  if (c_drag_0 <= 0) {
    return std::unexpected("c_drag_0 must be positive.");
  }

  if (c_drag_alpha <= 0) {
    return std::unexpected("c_drag_alpha must be positive.");
  }

  if (c_side_beta >= 0) {
    return std::unexpected("c_side_beta must be negative.");
  }

  if (c_roll_beta >= 0) {
    return std::unexpected("c_roll_beta must be negative.");
  }

  if (c_roll_p >= 0) {
    return std::unexpected("c_roll_p must be negative.");
  }

  if (c_pitch_alpha >= 0) {
    return std::unexpected("c_pitch_alpha must be negative.");
  }

  if (c_pitch_q >= 0) {
    return std::unexpected("c_pitch_q must be negative.");
  }

  if (c_yaw_r >= 0) {
    return std::unexpected("c_yaw_r must be negative.");
  }

  return {};
}

std::expected<void, std::string> AerodynamicCoefficients::load(const YAML::Node& node)
{
  if (!node.IsDefined() || !node.IsMap()) {
    return std::unexpected("Configuration node must be a map.");
  }

  if (const auto result = yaml::load(kCLift0Key, node, c_lift_0); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCLiftAlphaKey, node, c_lift_alpha); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCDrag0Key, node, c_drag_0); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCDragAlphaKey, node, c_drag_alpha); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCSideBetaKey, node, c_side_beta); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCRollBetaKey, node, c_roll_beta); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCRollPKey, node, c_roll_p); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCRollRKey, node, c_roll_r); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCPitch0Key, node, c_pitch_0); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCPitchAlphaKey, node, c_pitch_alpha); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCPitchAbsBetaKey, node, c_pitch_abs_beta); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCPitchAlphaRateKey, node, c_pitch_alpha_rate); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCPitchQKey, node, c_pitch_q); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCYawBetaKey, node, c_yaw_beta); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCYawPKey, node, c_yaw_p); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCYawRKey, node, c_yaw_r); !result) {
    return result;
  }

  return {};
}

YAML::Node AerodynamicCoefficients::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kCLift0Key] = yaml::format(c_lift_0);
  node[kCLiftAlphaKey] = yaml::format(c_lift_alpha);
  node[kCDrag0Key] = yaml::format(c_drag_0);
  node[kCDragAlphaKey] = yaml::format(c_drag_alpha);
  node[kCSideBetaKey] = yaml::format(c_side_beta);
  node[kCRollBetaKey] = yaml::format(c_roll_beta);
  node[kCRollPKey] = yaml::format(c_roll_p);
  node[kCRollRKey] = yaml::format(c_roll_r);
  node[kCPitch0Key] = yaml::format(c_pitch_0);
  node[kCPitchAlphaKey] = yaml::format(c_pitch_alpha);
  node[kCPitchAbsBetaKey] = yaml::format(c_pitch_abs_beta);
  node[kCPitchAlphaRateKey] = yaml::format(c_pitch_alpha_rate);
  node[kCPitchQKey] = yaml::format(c_pitch_q);
  node[kCYawBetaKey] = yaml::format(c_yaw_beta);
  node[kCYawPKey] = yaml::format(c_yaw_p);
  node[kCYawRKey] = yaml::format(c_yaw_r);

  return node;
}
}  // namespace tobas
