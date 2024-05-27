#include "../include/tobas_linear_control/equations.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
LinearEquation LinearEquation::scale(const VectorXd& scale) const
{
  assert(scale.rows() == variableSize());
  assert((scale.array() > 0).all());

  auto res = *this;
  for (Index c = 0; c < variableSize(); ++c)
    res.A.col(c) *= scale(c);

  return res;
}

LinearEquation LinearEquation::discretise(const double& dt) const
{
  assert(dt >= 0);

  auto res = *this;
  res.b *= dt;  // 単純に方程式の右辺にdtがかかるだけ
  return res;
}

ostream& operator<<(ostream& os, const LinearEquation& arg)
{
  os << "A:" << endl;
  os << arg.A << endl;
  os << "b:" << endl;
  os << arg.b << endl;

  return os;
}
}  // namespace ctrl
