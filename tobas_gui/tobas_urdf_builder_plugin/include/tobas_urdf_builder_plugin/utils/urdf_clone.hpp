// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <urdf_model/joint.h>
#include <urdf_model/link.h>

namespace tobas::gui::ub::utils
{
/**
 * Default clone function.
 * If instance variables contain pointers, `make_shared` does not copy the pointees,
 * so specialization or overloading is required.
 */
template <typename T>
std::shared_ptr<T> clone(const std::shared_ptr<T>& _ptr)
{
  if (!_ptr) {
    return nullptr;
  }

  return std::make_shared<T>(*_ptr);
}

::urdf::GeometrySharedPtr clone(const ::urdf::GeometrySharedPtr& _geometry);
::urdf::VisualSharedPtr clone(const ::urdf::VisualSharedPtr& _visual);
::urdf::CollisionSharedPtr clone(const ::urdf::CollisionSharedPtr& _collision);
::urdf::JointCalibrationSharedPtr clone(const ::urdf::JointCalibrationSharedPtr& _calibration);
::urdf::JointSharedPtr clone(const ::urdf::JointSharedPtr& _joint);
::urdf::LinkSharedPtr clone(const ::urdf::LinkSharedPtr& _link);
}  // namespace tobas::gui::ub::utils
