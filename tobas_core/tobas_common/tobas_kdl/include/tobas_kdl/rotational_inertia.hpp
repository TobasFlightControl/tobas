// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/rotationalinertia.hpp
// Modified by Tobas, Inc.

// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#pragma once

#include <expected>
#include <string>

#include "./frames.hpp"

namespace tobas::kdl
{
class RotationalInertia
{
public:
  Eigen::Matrix3d data;

  inline explicit RotationalInertia();
  inline explicit RotationalInertia(double _ixx, double _iyy, double _izz, double _ixy, double _ixz, double _iyz);
  inline explicit RotationalInertia(const Eigen::Matrix3d& _data);

  static inline RotationalInertia Zero();

  std::expected<void, std::string> validate() const;

  inline double ixx() const;
  inline double iyy() const;
  inline double izz() const;
  inline double ixy() const;
  inline double ixz() const;
  inline double iyz() const;

  inline double trace() const;

  inline RotationalInertia operator+(const RotationalInertia& _rhs) const;
  inline RotationalInertia& operator+=(const RotationalInertia& _rhs);

  /** Compute the angular momentum resulting from a rotational velocity omega. */
  inline Vector operator*(const Vector& _omega) const;

  inline friend RotationalInertia operator*(double _a, const RotationalInertia& _i);
  /** Compute Ia = Rab Ib Rab^T. */
  inline friend RotationalInertia operator*(const Rotation& _r_a_b, const RotationalInertia& _i_b);

  inline friend std::ostream& operator<<(std::ostream& _os, const RotationalInertia& _arg);
};

inline RotationalInertia::RotationalInertia() : RotationalInertia(0, 0, 0, 0, 0, 0)
{
}

inline RotationalInertia::RotationalInertia(double _ixx, double _iyy, double _izz, double _ixy, double _ixz, double _iyz)
{
  data << _ixx, _ixy, _ixz, _ixy, _iyy, _iyz, _ixz, _iyz, _izz;
}

inline RotationalInertia::RotationalInertia(const Eigen::Matrix3d& _data) : data(_data)
{
}

inline RotationalInertia RotationalInertia::Zero()
{
  return RotationalInertia(0, 0, 0, 0, 0, 0);
}

inline double RotationalInertia::ixx() const
{
  return data(0, 0);
}

inline double RotationalInertia::iyy() const
{
  return data(1, 1);
}

inline double RotationalInertia::izz() const
{
  return data(2, 2);
}

inline double RotationalInertia::ixy() const
{
  return data(0, 1);
}

inline double RotationalInertia::ixz() const
{
  return data(0, 2);
}

inline double RotationalInertia::iyz() const
{
  return data(1, 2);
}

inline double RotationalInertia::trace() const
{
  return data.trace();
}

inline RotationalInertia RotationalInertia::operator+(const RotationalInertia& _rhs) const
{
  return RotationalInertia(data + _rhs.data);
}

inline RotationalInertia& RotationalInertia::operator+=(const RotationalInertia& _rhs)
{
  data += _rhs.data;
  return *this;
}

inline Vector RotationalInertia::operator*(const Vector& _omega) const
{
  return Vector(data * _omega.data);
}

inline RotationalInertia operator*(double _a, const RotationalInertia& _i)
{
  return RotationalInertia(_a * _i.data);
}

inline RotationalInertia operator*(const Rotation& _r_a_b, const RotationalInertia& _i_b)
{
  return RotationalInertia(_r_a_b.data * _i_b.data * _r_a_b.data.transpose());
}

inline std::ostream& operator<<(std::ostream& _os, const RotationalInertia& _arg)
{
  _os << "Ixx: " << _arg.ixx() << ", Iyy: " << _arg.iyy() << ", Izz: " << _arg.izz() << ", Ixy: " << _arg.ixy()
      << ", Iyz: " << _arg.iyz() << ", Izx: " << _arg.ixz();
  return _os;
}
}  // namespace tobas::kdl
