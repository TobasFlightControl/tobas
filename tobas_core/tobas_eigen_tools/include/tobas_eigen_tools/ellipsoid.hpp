#pragma once

#include <eigen3/Eigen/Core>

#include "./typedef.hpp"

namespace eigen
{
/* axx x^2 + ayy y^2 + azz z^2 + 2 axy xy + 2 ayz yz + 2 azx zx + bx x + by y + bz z + c = 0 */
struct EllipsoidCoefficients
{
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
};

/**
 * @brief 任意の楕円体 (x^T A x + b^T x + c = 0) を原点中心の単位球に射影する．
 * https://rikei-tawamure.com/entry/2021/09/27/111205
 */
class Ellipsoid
{
public:
  explicit Ellipsoid();

  bool initialize(const EllipsoidCoefficients& coefs);

  /* 単位球に設定． */
  void setIdentity();

  const Eigen::Vector3d& getHardBias() const;
  void setHardBias(const Eigen::Vector3d& b);

  Eigen::Vector6d getSoftBias() const;
  void setSoftBias(const Eigen::Vector6d& t);

  inline Eigen::Vector3d fromUnitSphere(const Eigen::Vector3d& x) const;
  inline Eigen::Vector3d toUnitSphere(const Eigen::Vector3d& x) const;

private:
  Eigen::Vector3d b_;      // Hard-iron bias
  Eigen::Matrix3d T_;      // Soft-iron bias
  Eigen::Matrix3d T_inv_;  // Soft-iron bias (inverse)
};

inline Eigen::Vector3d Ellipsoid::fromUnitSphere(const Eigen::Vector3d& x) const
{
  return T_ * (x + b_);
}

inline Eigen::Vector3d Ellipsoid::toUnitSphere(const Eigen::Vector3d& x) const
{
  return T_inv_ * (x - b_);
}
}  // namespace eigen
