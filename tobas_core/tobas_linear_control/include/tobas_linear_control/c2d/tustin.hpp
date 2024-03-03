#pragma once

#include "./base.hpp"

namespace ctrl
{
/**
 * @brief 双一次変換．
 * https://www.dropbox.com/s/ijfnlkvcep1w0f2/%E5%A7%BF%E5%8B%A2%E6%8E%A8%E5%AE%9A%E3%81%AE%E5%9F%BA%E7%A4%8E.pdf
 */
class C2D_Tustin : BaseC2D
{
public:
  explicit C2D_Tustin(const size_t& x_size, const size_t& u_size);
  explicit C2D_Tustin();

  LinearDynamics convert(const LinearDynamics& cont, const double& dt) override;

  void resize(const size_t& x_size, const size_t& u_size);

private:
  size_t x_size_;
  size_t u_size_;
  Eigen::MatrixXd I_;  // 単位行列
};
}  // namespace ctrl
