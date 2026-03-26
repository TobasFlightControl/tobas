#include "tobas_kdl/vector.hpp"

#include <iostream>

#include <tobas_math/core.hpp>

using namespace std;

namespace tobas
{
namespace kdl
{
bool Vector::isParallel(const Vector& rhs, bool same_direction_only, double angle_tol_rad, double zero_tol) const
{
  assert(0. < angle_tol_rad && angle_tol_rad < 1.);
  assert(zero_tol > 0.);

  const auto na2 = this->squaredNorm();
  const auto nb2 = rhs.squaredNorm();

  // ゼロベクトルは平行が定義できないため負荷
  if (na2 < zero_tol || nb2 < zero_tol) {
    cerr << "Parallelism is undefined for zero vector." << endl;
    return false;
  }

  const auto dot = this->dot(rhs);
  const auto cos2 = math::sqr(dot) / (na2 * nb2);
  const auto thresh = math::sqr(cos(angle_tol_rad));

  if (same_direction_only) {
    return cos2 > thresh && dot > 0.;
  }
  else {
    return cos2 > thresh;
  }
}
}  // namespace kdl
}  // namespace tobas
