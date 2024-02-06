#include "../include/tobas_kdl/polar.hpp"

namespace KDL
{
SphericalCoordinate::SphericalCoordinate()
{
}

SphericalCoordinate::SphericalCoordinate(const double& rho, const double& phi, const double& theta)
  : rho(rho), phi(phi), theta(theta)
{
}

SphericalCoordinate SphericalCoordinate::Cartesian(const Vector& cart)
{
  const auto r = sqrt(sqr(cart.x()) + sqr(cart.y()));
  return SphericalCoordinate(cart.norm(), atan2(r, cart.z()), atan2(cart.y(), cart.x()));
}

Vector SphericalCoordinate::toCartesian() const
{
  return rho * Vector(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));
}
}  // namespace KDL
