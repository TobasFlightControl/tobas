#pragma once

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

  explicit SphericalCoordinate();
  explicit SphericalCoordinate(const double& _rho, const double& _phi, const double& _theta);

  static SphericalCoordinate Cartesian(const Vector& cart);

  Vector toCartesian() const;
};
}  // namespace kdl
