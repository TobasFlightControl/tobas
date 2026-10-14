// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./vector.hpp"

namespace tobas
{
namespace kdl
{
/**
 * Spherical coordinate system.
 *
 * @see https://wiis.info/math/euclidean-space/euclidean-space/spherical-coordinates/
 */
class SphericalCoordinate
{
public:
  double rho, phi, theta;

  inline explicit SphericalCoordinate();
  inline explicit SphericalCoordinate(double _rho, double _phi, double _theta);

  static inline SphericalCoordinate Cartesian(const Vector& _cart);

  inline Vector toCartesian() const;

  inline friend std::ostream& operator<<(std::ostream& _os, const SphericalCoordinate& _arg);
};

inline SphericalCoordinate::SphericalCoordinate()
{
}

inline SphericalCoordinate::SphericalCoordinate(double _rho, double _phi, double _theta)
  : rho(_rho), phi(_phi), theta(_theta)
{
}

inline SphericalCoordinate SphericalCoordinate::Cartesian(const Vector& _cart)
{
  const auto r = std::hypot(_cart.x(), _cart.y());
  return SphericalCoordinate(_cart.norm(), std::atan2(r, _cart.z()), std::atan2(_cart.y(), _cart.x()));
}

inline Vector SphericalCoordinate::toCartesian() const
{
  return rho * Vector(std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta), std::cos(phi));
}

inline std::ostream& operator<<(std::ostream& _os, const SphericalCoordinate& _arg)
{
  _os << "rho: " << _arg.rho << ", phi: " << _arg.phi << ", theta: " << _arg.theta;
}
}  // namespace kdl
}  // namespace tobas
