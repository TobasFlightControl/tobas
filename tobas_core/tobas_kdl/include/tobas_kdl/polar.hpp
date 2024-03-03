#pragma once

#include "./vector.hpp"

namespace KDL
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
  explicit SphericalCoordinate(const double& rho, const double& phi, const double& theta);

  static SphericalCoordinate Cartesian(const Vector& cart);

  Vector toCartesian() const;
};
}  // namespace KDL
