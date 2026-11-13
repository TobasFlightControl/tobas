// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/rigidbodyinertia.cpp
// Modified by Tobas, Inc.

// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.

// You should have received a copy of the GNU General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "tobas_kdl/rigid_body_inertia.hpp"

namespace tobas::kdl
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
}  // namespace tobas::kdl
