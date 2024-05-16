#include "../include/tobas_kdl/vector.hpp"
#include "../include/tobas_kdl/utilities/utility.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_kdl
{
double Vector::norm(double eps) const
{
  double tmp1 = fabs(data.x());
  double tmp2 = fabs(data.y());
  if (tmp1 >= tmp2)
  {
    tmp2 = fabs(data.z());
    if (tmp1 >= tmp2)
    {
      if (tmp1 < eps)
      {
        // only to everything exactly zero case, all other are handled correctly
        return 0;
      }
      return tmp1 * sqrt(1 + sqr(data.y() / data.x()) + sqr(data.z() / data.x()));
    }
    else
    {
      return tmp2 * sqrt(1 + sqr(data.x() / data.z()) + sqr(data.y() / data.z()));
    }
  }
  else
  {
    tmp1 = fabs(data.z());
    if (tmp2 > tmp1)
    {
      return tmp2 * sqrt(1 + sqr(data.x() / data.y()) + sqr(data.z() / data.y()));
    }
    else
    {
      return tmp1 * sqrt(1 + sqr(data.x() / data.z()) + sqr(data.y() / data.z()));
    }
  }
}

double Vector::normalize(double eps)
{
  const double v = this->norm();
  if (v < eps)
  {
    *this = Vector(1, 0, 0);
    return 0;
  }
  else
  {
    *this = (*this) / v;
    return v;
  }
}

Vector Vector::normalized() const
{
  return Vector(data.normalized());
}
}  // namespace tobas_kdl
