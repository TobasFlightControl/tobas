#pragma once

#include <boost/array.hpp>

#include "./base.hpp"

namespace ctrl
{
class C2D_RK4 : BaseC2D
{
public:
  explicit C2D_RK4(const size_t& x_size, const size_t& u_size);
  explicit C2D_RK4();

  LinearDynamics convert(const LinearDynamics& cont, const double& dt) override;

  void resize(const size_t& x_size, const size_t& u_size);

private:
  size_t x_size_;
  size_t u_size_;
  boost::array<Eigen::MatrixXd, 5> Ac_dt_pows_;  // Ac*dtの累乗を保持する配列
  boost::array<double, 5> factorials_;           // 階乗を保持する配列
};
}  // namespace ctrl
