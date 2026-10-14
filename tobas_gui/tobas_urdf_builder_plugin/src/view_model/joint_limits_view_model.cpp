// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/view_model/joint_limits_view_model.hpp"

namespace tobas::gui::ub::view_model
{
void JointLimitsViewModel::sync()
{
}

double JointLimitsViewModel::lower() const
{
  return model_->lower;
}

void JointLimitsViewModel::lower(double _lower)
{
  model_->lower = _lower;
}

double JointLimitsViewModel::upper() const
{
  return model_->upper;
}

void JointLimitsViewModel::upper(double _upper)
{
  model_->upper = _upper;
}

double JointLimitsViewModel::effort() const
{
  return model_->effort;
}

void JointLimitsViewModel::effort(double _effort)
{
  model_->effort = _effort;
}

double JointLimitsViewModel::velocity() const
{
  return model_->velocity;
}

void JointLimitsViewModel::velocity(double _velocity)
{
  model_->velocity = _velocity;
}
}  // namespace tobas::gui::ub::view_model
