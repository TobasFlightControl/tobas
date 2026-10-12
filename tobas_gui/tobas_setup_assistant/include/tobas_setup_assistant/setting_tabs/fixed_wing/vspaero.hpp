// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
/** Aerodynamic coefficients read from VSPAERO stability output. */
struct VspaeroData
{
  // Public members form a plain data value independent of the parser.
  double c_lift_0 = {};
  double c_lift_alpha = {};
  double c_drag_0 = {};
  double c_drag_alpha = {};
  double c_side_beta = {};
  double c_roll_beta = {};
  double c_roll_p = {};
  double c_roll_r = {};
  double c_pitch_0 = {};
  double c_pitch_alpha = {};
  double c_pitch_abs_beta = {};
  double c_pitch_alpha_rate = {};
  double c_pitch_q = {};
  double c_yaw_beta = {};
  double c_yaw_p = {};
  double c_yaw_r = {};
};

/** Return parsed coefficients or a diagnostic describing the failure. */
std::expected<VspaeroData, std::string> parseVspaero(const std::string& stab_path);
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
