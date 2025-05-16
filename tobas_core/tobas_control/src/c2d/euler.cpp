#include "tobas_control/c2d/euler.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
C2D_Euler::C2D_Euler(const Index& x_size, const Index& u_size)
  : x_size_(x_size), u_size_(u_size), I_(MatrixXd::Identity(x_size, x_size))
{
}

LinearDynamics C2D_Euler::convert(const LinearDynamics& cont, const double& dt)
{
  assert(cont.stateSize() == x_size_ && cont.inputSize() == u_size_);
  assert(cont.isFinite());
  assert(dt > 0);

  return LinearDynamics(I_ + cont.A * dt, cont.B * dt);
}
}  // namespace ctrl
