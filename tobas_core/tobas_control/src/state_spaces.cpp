#include "../include/tobas_control/state_spaces.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
LinearDynamics LinearDynamics::scale(const VectorXd& x_scale, const VectorXd& u_scale) const
{
  assert(x_scale.rows() == stateSize());
  assert(u_scale.rows() == inputSize());
  assert(eigen::isFinite(x_scale));
  assert(eigen::isFinite(u_scale));
  assert((x_scale.array() > 0).all());
  assert((u_scale.array() > 0).all());

  auto res = *this;

  // memo: 2-20
  for (Index c = 0; c < stateSize(); ++c)
    res.A.col(c) *= x_scale(c);
  for (Index r = 0; r < stateSize(); ++r)
    res.A.row(r) /= x_scale(r);
  for (Index c = 0; c < inputSize(); ++c)
    res.B.col(c) *= u_scale(c);
  for (Index r = 0; r < stateSize(); ++r)
    res.B.row(r) /= x_scale(r);

  return res;
}

ostream& operator<<(ostream& os, const LinearDynamics& arg)
{
  os << "A:" << endl;
  os << arg.A << endl;
  os << "B:" << endl;
  os << arg.B << endl;

  return os;
}

ostream& operator<<(ostream& os, const LinearStateSpace& arg)
{
  os << "A:" << endl;
  os << arg.A << endl;
  os << "B:" << endl;
  os << arg.B << endl;
  os << "C:" << endl;
  os << arg.C << endl;

  return os;
}
}  // namespace ctrl
