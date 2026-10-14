// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base_view_model.hpp"

namespace tobas::gui::ub::view_model
{
class JointLimitsViewModel : public BaseViewModel<::urdf::JointLimits, JointLimitsViewModel>
{
public:
  using BaseViewModel<::urdf::JointLimits, JointLimitsViewModel>::BaseViewModel;

  void sync() override;

  double lower() const;
  void lower(double _lower);

  double upper() const;
  void upper(double _upper);

  double effort() const;
  void effort(double _effort);

  double velocity() const;
  void velocity(double _velocity);
};

using JointLimitsViewModelPtr = std::shared_ptr<JointLimitsViewModel>;
}  // namespace tobas::gui::ub::view_model
