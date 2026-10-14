// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/view_model/inertial_view_model.hpp"

#include <cassert>

#include <tobas_math/core.hpp>

namespace tobas
{
namespace gui
{
namespace ub
{
namespace view_model
{
void InertialViewModel::sync()
{
}

const ::urdf::Pose& InertialViewModel::origin() const
{
  return model_->origin;
}

void InertialViewModel::origin(const ::urdf::Pose& _origin)
{
  model_->origin = _origin;
}

double InertialViewModel::mass() const
{
  return model_->mass;
}

void InertialViewModel::mass(double _mass)
{
  model_->mass = _mass;
}

Inertia InertialViewModel::inertia() const
{
  struct Inertia result;
  result.ixx = model_->ixx;
  result.ixy = model_->ixy;
  result.ixz = model_->ixz;
  result.iyy = model_->iyy;
  result.iyz = model_->iyz;
  result.izz = model_->izz;
  return result;
}

void InertialViewModel::inertia(const Inertia& _inertia)
{
  model_->ixx = _inertia.ixx;
  model_->ixy = _inertia.ixy;
  model_->ixz = _inertia.ixz;
  model_->iyy = _inertia.iyy;
  model_->iyz = _inertia.iyz;
  model_->izz = _inertia.izz;
}

void InertialViewModel::buildInertiaBox(double _x, double _y, double _z)
{
  assert(_x >= 0.0);
  assert(_y >= 0.0);
  assert(_z >= 0.0);

  model_->ixx = model_->mass * (math::sqr(_y) + math::sqr(_z)) / 12;
  model_->ixy = 0.0;
  model_->ixz = 0.0;
  model_->iyy = model_->mass * (math::sqr(_x) + math::sqr(_z)) / 12;
  model_->iyz = 0.0;
  model_->izz = model_->mass * (math::sqr(_x) + math::sqr(_y)) / 12;
}

void InertialViewModel::buildInertiaCylinder(double _radius, double _length)
{
  assert(_radius >= 0.0);
  assert(_length >= 0.0);

  model_->ixx = model_->mass * (math::sqr(_radius) / 4 + math::sqr(_length) / 12);
  model_->ixy = 0.0;
  model_->ixz = 0.0;
  model_->iyy = model_->ixx;
  model_->iyz = 0.0;
  model_->izz = model_->mass * math::sqr(_radius) / 2;
}

void InertialViewModel::buildInertiaSphere(double _radius)
{
  assert(_radius >= 0.0);

  model_->ixx = 0.4 * model_->mass * math::sqr(_radius);
  model_->ixy = 0.0;
  model_->ixz = 0.0;
  model_->iyy = model_->ixx;
  model_->iyz = 0.0;
  model_->izz = model_->ixx;
}
}  // namespace view_model
}  // namespace ub
}  // namespace gui
}  // namespace tobas
