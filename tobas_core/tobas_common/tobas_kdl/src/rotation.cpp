// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/frames.cpp
// Modified by Tobas, Inc.

/***************************************************************************
                        frames.cxx -  description
                       -------------------------
    begin                : June 2006
    copyright            : (C) 2006 Erwin Aertbelien
    email                : firstname.lastname@mech.kuleuven.ac.be

 History (only major changes)( AUTHOR-Description ) :

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include "tobas_kdl/rotation.hpp"

#include <tobas_eigen_tools/linalg.hpp>
#include <tobas_math/float.hpp>
#include <tobas_math/linalg.hpp>

namespace tobas::kdl
{
namespace
{
constexpr double kEps = 1e-12;
}  // namespace

Rotation Rotation::RotX(double _angle)
{
  const auto cs = std::cos(_angle);
  const auto sn = std::sin(_angle);
  return Rotation(1, 0, 0, 0, cs, -sn, 0, sn, cs);
}

Rotation Rotation::RotY(double _angle)
{
  const auto cs = std::cos(_angle);
  const auto sn = std::sin(_angle);
  return Rotation(cs, 0, sn, 0, 1, 0, -sn, 0, cs);
}

Rotation Rotation::RotZ(double _angle)
{
  const auto cs = std::cos(_angle);
  const auto sn = std::sin(_angle);
  return Rotation(cs, -sn, 0, sn, cs, 0, 0, 0, 1);
}

Rotation Rotation::Rot(const Vector& _axis, double _angle)
{
  // Axis must be normalized.
  assert(math::isClose(_axis.norm(), 1.0));

  // The formula
  // R(n, θ) = n n^T + (E - n n^T)) std::cos(θ) + skew(n) sin(θ) = E + skew(n) sin(θ) + skew(n)^2 (1 - std::cos(θ))
  // can be found by multiplying it with an arbitrary vector p and noting that this vector is rotated.
  const auto ct = std::cos(_angle);
  const auto st = std::sin(_angle);
  const auto vt = 1 - ct;
  const auto m_vt_0 = vt * _axis.x();
  const auto m_vt_1 = vt * _axis.y();
  const auto m_vt_2 = vt * _axis.z();
  const auto m_st_0 = _axis.x() * st;
  const auto m_st_1 = _axis.y() * st;
  const auto m_st_2 = _axis.z() * st;
  const auto m_vt_0_1 = m_vt_0 * _axis.y();
  const auto m_vt_0_2 = m_vt_0 * _axis.z();
  const auto m_vt_1_2 = m_vt_1 * _axis.z();

  return Rotation(
    ct + m_vt_0 * _axis.x(),
    -m_st_2 + m_vt_0_1,
    m_st_1 + m_vt_0_2,
    m_st_2 + m_vt_0_1,
    ct + m_vt_1 * _axis.y(),
    -m_st_0 + m_vt_1_2,
    -m_st_1 + m_vt_0_2,
    m_st_0 + m_vt_1_2,
    ct + m_vt_2 * _axis.z());
}

Rotation Rotation::Rot(const Vector& _vec)
{
  return Rotation::Rot(_vec.normalized(), _vec.norm());
}

Rotation Rotation::RPY(double _roll, double _pitch, double _yaw)
{
  const auto ca = std::cos(_yaw);
  const auto sa = std::sin(_yaw);
  const auto cb = std::cos(_pitch);
  const auto sb = std::sin(_pitch);
  const auto cc = std::cos(_roll);
  const auto sc = std::sin(_roll);

  const auto xx = ca * cb;
  const auto yx = ca * sb * sc - sa * cc;
  const auto zx = ca * sb * cc + sa * sc;
  const auto xy = sa * cb;
  const auto yy = sa * sb * sc + ca * cc;
  const auto zy = sa * sb * cc - ca * sc;
  const auto xz = -sb;
  const auto yz = cb * sc;
  const auto zz = cb * cc;

  return Rotation(xx, yx, zx, xy, yy, zy, xz, yz, zz);
}

Rotation Rotation::Quaternion(double _x, double _y, double _z, double _w)
{
  assert(math::isClose(math::norm(_x, _y, _z, _w), 1.0));

  const auto tx = 2 * _x;
  const auto ty = 2 * _y;
  const auto tz = 2 * _z;
  const auto twx = tx * _w;
  const auto twy = ty * _w;
  const auto twz = tz * _w;
  const auto txx = tx * _x;
  const auto txy = ty * _x;
  const auto txz = tz * _x;
  const auto tyy = ty * _y;
  const auto tyz = tz * _y;
  const auto tzz = tz * _z;

  return Rotation(
    1 - (tyy + tzz), txy - twz, txz + twy, txy + twz, 1 - (txx + tzz), tyz - twx, txz - twy, tyz + twx, 1 - (txx + tyy));
}

std::expected<void, std::string> Rotation::validate() const
{
  if (!eigen::isSpecialOrthogonal(data)) {
    return std::unexpected("Rotation matrix must belong to SO(3).");
  }

  return {};
}

void Rotation::getQuaternion(double& _x, double& _y, double& _z, double& _w) const
{
  const auto trace = data.trace();
  if (trace > kEps) {
    const auto s = 0.5 / std::sqrt(trace + 1.0);
    _w = 0.25 / s;
    _x = (data(2, 1) - data(1, 2)) * s;
    _y = (data(0, 2) - data(2, 0)) * s;
    _z = (data(1, 0) - data(0, 1)) * s;
  }
  else {
    if (data(0, 0) > data(1, 1) && data(0, 0) > data(2, 2)) {
      const auto s = 2.0 * std::sqrt(1.0 + data(0, 0) - data(1, 1) - data(2, 2));
      _w = (data(2, 1) - data(1, 2)) / s;
      _x = 0.25 * s;
      _y = (data(0, 1) + data(1, 0)) / s;
      _z = (data(0, 2) + data(2, 0)) / s;
    }
    else if (data(1, 1) > data(2, 2)) {
      const auto s = 2.0 * std::sqrt(1.0 + data(1, 1) - data(0, 0) - data(2, 2));
      _w = (data(0, 2) - data(2, 0)) / s;
      _x = (data(0, 1) + data(1, 0)) / s;
      _y = 0.25 * s;
      _z = (data(1, 2) + data(2, 1)) / s;
    }
    else {
      const auto s = 2.0 * std::sqrt(1.0 + data(2, 2) - data(0, 0) - data(1, 1));
      _w = (data(1, 0) - data(0, 1)) / s;
      _x = (data(0, 2) + data(2, 0)) / s;
      _y = (data(1, 2) + data(2, 1)) / s;
      _z = 0.25 * s;
    }
  }
}

void Rotation::getRPY(double& _roll, double& _pitch, double& _yaw) const
{
  _pitch = getPitch();
  if (std::abs(_pitch) > (M_PI_2 - kEps)) {
    _yaw = std::atan2(-data(0, 1), data(1, 1));
    _roll = 0.0;
  }
  else {
    _roll = std::atan2(data(2, 1), data(2, 2));
    _yaw = std::atan2(data(1, 0), data(0, 0));
  }
}

std::tuple<double, double, double> Rotation::getRPY() const
{
  double roll, pitch, yaw;
  getRPY(roll, pitch, yaw);
  return { roll, pitch, yaw };
}

Vector Rotation::getRot() const
{
  const auto [angle, axis] = Rotation::getAngleAxis();
  return angle * axis;
}

std::pair<double, Vector> Rotation::getAngleAxis() const
{
  // Optional check that input is pure rotation, 'isRotationMatrix' is defined at:
  // Ref: http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/

  if (
    std::abs(data(0, 1) - data(1, 0) < kEps) && std::abs(data(0, 2) - data(2, 0)) < kEps &&
    std::abs(data(1, 2) - data(2, 1)) < kEps) {
    // Singularity found.
    // First check for identity matrix which must have +1
    // for all terms in leading diagonal and zero in other terms.
    constexpr auto kEps_10 = kEps * 10;  // Margin to distinguish between 0 and 180 degrees.
    if (
      std::abs(data(0, 1) + data(1, 0)) < kEps_10 && std::abs(data(0, 2) + data(2, 0)) < kEps_10 &&
      std::abs(data(1, 2) + data(2, 1)) < kEps_10 && std::abs(trace() - 3) < kEps_10) {
      // This singularity is the identity matrix, so the angle is 0 and the axis is arbitrary.
      return { 0.0, Vector::UnitZ() };
    }

    // Otherwise this singularity is angle = 180.
    const auto xx = (data(0, 0) + 1) / 2;
    const auto yy = (data(1, 1) + 1) / 2;
    const auto zz = (data(2, 2) + 1) / 2;
    const auto xy = (data(0, 1) + data(1, 0)) / 4;
    const auto xz = (data(0, 2) + data(2, 0)) / 4;
    const auto yz = (data(1, 2) + data(2, 1)) / 4;

    double x, y, z;
    if (xx > yy && xx > zz) {
      // data(0, 0) is the largest diagonal term.
      x = std::sqrt(xx);
      y = xy / x;
      z = xz / x;
    }
    else if (yy > zz) {
      // data(1, 1) is the largest diagonal term.
      y = std::sqrt(yy);
      x = xy / y;
      z = yz / y;
    }
    else {
      // data(2, 2) is the largest diagonal term so base result on this.
      z = std::sqrt(zz);
      x = xz / z;
      y = yz / z;
    }

    return { M_PI, Vector(x, y, z) };  // return 180 deg rotation.
  }

  const auto f = (data.trace() - 1) / 2;

  const auto x = data(2, 1) - data(1, 2);
  const auto y = data(0, 2) - data(2, 0);
  const auto z = data(1, 0) - data(0, 1);
  const Vector axis(x, y, z);
  const auto angle = std::atan2(axis.norm() / 2, f);
  return { angle, axis.normalized() };
}
}  // namespace tobas::kdl
