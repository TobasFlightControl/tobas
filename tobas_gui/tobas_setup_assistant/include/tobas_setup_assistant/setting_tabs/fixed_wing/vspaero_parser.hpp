// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
class VSPAEROParser
{
public:
  bool parse(const std::string& stab_path);

  double c_lift_0() const;
  double c_lift_alpha() const;
  double c_drag_0() const;
  double c_drag_alpha() const;
  double c_side_beta() const;
  double c_roll_beta() const;
  double c_roll_p() const;
  double c_roll_r() const;
  double c_pitch_0() const;
  double c_pitch_alpha() const;
  double c_pitch_abs_beta() const;
  double c_pitch_alpha_rate() const;
  double c_pitch_q() const;
  double c_yaw_beta() const;
  double c_yaw_p() const;
  double c_yaw_r() const;

private:
  double c_lift_0_;
  double c_lift_alpha_;
  double c_drag_0_;
  double c_drag_alpha_;
  double c_side_beta_;
  double c_roll_beta_;
  double c_roll_p_;
  double c_roll_r_;
  double c_pitch_0_;
  double c_pitch_alpha_;
  double c_pitch_abs_beta_;
  double c_pitch_alpha_rate_;
  double c_pitch_q_;
  double c_yaw_beta_;
  double c_yaw_p_;
  double c_yaw_r_;
};
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
