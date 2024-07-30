#include <eigen3/Eigen/LU>  // inverse()に必要

#include "../../include/tobas_linear_control/c2d/tustin.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
C2D_Tustin::C2D_Tustin(const Index& x_size, const Index& u_size)
{
  resize(x_size, u_size);
}

C2D_Tustin::C2D_Tustin()
{
}

LinearDynamics C2D_Tustin::convert(const LinearDynamics& cont, const double& dt)
{
  assert(cont.stateSize() == x_size_ && cont.inputSize() == u_size_);
  assert(cont.isFinite());
  assert(dt > 0);

  const MatrixXd dt2A = (dt / 2) * cont.A;
  const MatrixXd I_plus_dt2A = I_ + dt2A;
  const MatrixXd I_minus_dt2A = I_ - dt2A;
  const PartialPivLU<MatrixXd> lu(I_minus_dt2A);

  LinearDynamics res;
  res.A = lu.solve(I_plus_dt2A);
  res.B = lu.solve(cont.B * dt);

  return res;
}

void C2D_Tustin::resize(const Index& x_size, const Index& u_size)
{
  x_size_ = x_size;
  u_size_ = u_size;
  I_ = MatrixXd::Identity(x_size, x_size);
}
}  // namespace ctrl
