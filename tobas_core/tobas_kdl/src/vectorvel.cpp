#include "../include/tobas_kdl/vectorvel.hpp"

namespace kdl
{
doubleVel VectorVel::norm(double eps) const
{
  const auto n = p.norm(eps);
  if (n < eps)  // Setting norm  of p and v to 0 in case norm of p is smaller than eps
    return doubleVel(0, 0);
  return doubleVel(n, p.dot(v) / n);
}

VectorVel& VectorVel::operator+=(const VectorVel& rhs)
{
  p += rhs.p;
  v += rhs.v;
  return *this;
}

VectorVel& VectorVel::operator-=(const VectorVel& rhs)
{
  p -= rhs.p;
  v -= rhs.v;
  return *this;
}
}  // namespace kdl
