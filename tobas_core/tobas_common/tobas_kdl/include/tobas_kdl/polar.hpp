#pragma once

#include <tobas_math/linalg.hpp>

#include "./vector.hpp"

namespace kdl
{
/**
 * @brief 球面座標系．
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
  const auto r = math::norm(cart.x(), cart.y());
  return SphericalCoordinate(cart.norm(), atan2(r, cart.z()), atan2(cart.y(), cart.x()));
}

inline Vector SphericalCoordinate::toCartesian() const
{
  return rho * Vector(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));
}

inline std::ostream& operator<<(std::ostream& os, const SphericalCoordinate& arg)
{
  os << "rho: " << arg.rho << ", phi: " << arg.phi << ", theta: " << arg.theta;
}
}  // namespace kdl
