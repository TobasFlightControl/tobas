// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include "./base_view_model.hpp"

namespace tobas::gui::ub::view_model
{
struct Inertia
{
  double ixx, ixy, ixz;
  double iyy, iyz;
  double izz;
};

class InertialViewModel : public BaseViewModel<::urdf::Inertial, InertialViewModel>
{
public:
  using BaseViewModel<::urdf::Inertial, InertialViewModel>::BaseViewModel;

  void sync() override;

  const ::urdf::Pose& origin() const;
  void origin(const ::urdf::Pose& _origin);

  double mass() const;
  void mass(double _mass);

  Inertia inertia() const;
  void inertia(const Inertia& _inertia);

  /** Inertia wrt. CoM of a box. */
  void buildInertiaBox(double _x, double _y, double _z);

  /** Inertia wrt. CoM of a cylinder. */
  void buildInertiaCylinder(double _radius, double _length);

  /** Inertia wrt. CoM of a sphere. */
  void buildInertiaSphere(double _radius);
};

using InertialViewModelPtr = std::shared_ptr<InertialViewModel>;
}  // namespace tobas::gui::ub::view_model
