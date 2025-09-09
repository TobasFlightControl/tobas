#pragma once

#include <array>

#include "./base.hpp"

namespace ctrl
{
class C2D_RK4 : BaseC2D
{
public:
  explicit C2D_RK4(const Eigen::Index& x_size, const Eigen::Index& u_size);
  explicit C2D_RK4();

  LinearDynamics convert(const LinearDynamics& cont, const double& dt) override;

  void resize(const Eigen::Index& x_size, const Eigen::Index& u_size);

private:
  Eigen::Index x_size_, u_size_;
  std::array<Eigen::MatrixXd, 5> Ac_dt_pows_;  // Ac*dtの累乗を保持する配列
  std::array<double, 5> factorials_;           // 階乗を保持する配列
};
}  // namespace ctrl
