#pragma once

#include "./base.hpp"

namespace ctrl
{
class C2D_Euler : BaseC2D
{
public:
  explicit C2D_Euler(const size_t& x_size, const size_t& u_size);

  LinearDynamics convert(const LinearDynamics& cont, const double& dt) override;

private:
  const size_t x_size_;
  const size_t u_size_;
  const Eigen::MatrixXd I_;  // 単位行列
};
}  // namespace ctrl
