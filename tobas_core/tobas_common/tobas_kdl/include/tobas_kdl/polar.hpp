// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./vector.hpp"

namespace tobas
{
namespace kdl
{
/**
 * @brief Spherical coordinate system.
 * cf. https://wiis.info/math/euclidean-space/euclidean-space/spherical-coordinates/
 */
class SphericalCoordinate
{
public:
  double rho, phi, theta;

  inline explicit SphericalCoordinate();
  inline explicit SphericalCoordinate(double _rho, double _phi, double _theta);

  static inline SphericalCoordinate Cartesian(const Vector& cart);

  inline Vector toCartesian() const;

  inline friend std::ostream& operator<<(std::ostream& os, const SphericalCoordinate& arg);
};

inline SphericalCoordinate::SphericalCoordinate()
{
}

inline SphericalCoordinate::SphericalCoordinate(double _rho, double _phi, double _theta)
  : rho(_rho), phi(_phi), theta(_theta)
{
}

inline SphericalCoordinate SphericalCoordinate::Cartesian(const Vector& cart)
{
  const auto r = std::hypot(cart.x(), cart.y());
  return SphericalCoordinate(cart.norm(), std::atan2(r, cart.z()), std::atan2(cart.y(), cart.x()));
}

inline Vector SphericalCoordinate::toCartesian() const
{
  return rho * Vector(std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta), std::cos(phi));
}

inline std::ostream& operator<<(std::ostream& os, const SphericalCoordinate& arg)
{
  os << "rho: " << arg.rho << ", phi: " << arg.phi << ", theta: " << arg.theta;
}
}  // namespace kdl
}  // namespace tobas
