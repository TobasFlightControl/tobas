#include "tobas_control/c2d/rk4.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
C2D_RK4::C2D_RK4(const Index& x_size, const Index& u_size)
{
  resize(x_size, u_size);
}

C2D_RK4::C2D_RK4()
{
}

LinearDynamics C2D_RK4::convert(const LinearDynamics& cont, const double& dt)
{
  assert(cont.stateSize() == x_size_ && cont.inputSize() == u_size_);
  assert(cont.isFinite());
  assert(dt >= 0);

  // Ac_dtの累乗を計算
  const MatrixXd Ac_dt = cont.A * dt;
  for (size_t i = 1; i <= 4; ++i) {
    Ac_dt_pows_[i] = Ac_dt_pows_[i - 1] * Ac_dt;
  }

  // Adを計算
  MatrixXd Ad = MatrixXd::Identity(x_size_, x_size_);
  for (size_t i = 1; i <= 4; ++i) {
    Ad += Ac_dt_pows_[i] / factorials_[i];
  }

  // Bdを計算
  MatrixXd Bd = MatrixXd::Identity(x_size_, x_size_);
  for (size_t i = 1; i <= 3; ++i) {
    Bd += Ac_dt_pows_[i] / factorials_[i + 1];
  }
  Bd *= (cont.B * dt);

  return LinearDynamics(Ad, Bd);
}

void C2D_RK4::resize(const Index& x_size, const Index& u_size)
{
  x_size_ = x_size;
  u_size_ = u_size;

  Ac_dt_pows_[0] = MatrixXd::Identity(x_size_, x_size_);

  factorials_[0] = 1;
  for (size_t i = 1; i <= 4; ++i) {
    factorials_[i] = factorials_[i - 1] * static_cast<double>(i);
  }
}
}  // namespace ctrl
