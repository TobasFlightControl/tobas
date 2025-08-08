#pragma once

#include <eigen3/Eigen/Core>

namespace math
{
/**
 * @brief 任意の楕円体 (x^T A x + b^T x + c = 0) を原点中心の単位球に射影する．
 * https://rikei-tawamure.com/entry/2021/09/27/111205
 */
class EllipseTransformer
{
public:
  double a_xx;
  double a_yy;
  double a_zz;
  double a_xy;
  double a_yz;
  double a_zx;
  double b_x;
  double b_y;
  double b_z;
  double c;

  explicit EllipseTransformer();

  void setIdentity();
  bool initialize();

  inline Eigen::Vector3d transform(const Eigen::Vector3d& x) const;

  friend std::ostream& operator<<(std::ostream& os, const EllipseTransformer& arg);

private:
  Eigen::Vector3d b_;      // Hard-iron bias
  Eigen::Matrix3d T_inv_;  // Soft-iron bias (inverse)
};

inline Eigen::Vector3d EllipseTransformer::transform(const Eigen::Vector3d& x) const
{
  // xm = T xt + b <=> xt = T^(-1) (xm - b)
  return T_inv_ * (x - b_);
}
}  // namespace math
