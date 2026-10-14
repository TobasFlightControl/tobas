// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/rigid_body_inertia.hpp"

namespace tobas
{
namespace kdl
{
RigidBodyInertia::RigidBodyInertia(double _m, const Vector& _oc, const RotationalInertia& _ic) : m_(_m), h_(_m * _oc)
{
  const auto& c_eig = _oc.data;
  Eigen::Matrix3d tmp = c_eig * c_eig.transpose();
  tmp.diagonal().array() -= c_eig.dot(c_eig);
  I_.data = _ic.data - _m * tmp;
}

std::expected<void, std::string> RigidBodyInertia::validate() const
{
  if (m_ < 0.0) {
    return std::unexpected("Mass must be non-negative.");
  }

  return I_.validate();
}

std::ostream& operator<<(std::ostream& _os, const RigidBodyInertia& _arg)
{
  _os << "Mass: " << _arg.m_ << '\n';
  _os << "Spatial Momentum: " << _arg.h_ << '\n';
  _os << "Rotational Inertia: " << _arg.I_;
  return _os;
}
}  // namespace kdl
}  // namespace tobas
