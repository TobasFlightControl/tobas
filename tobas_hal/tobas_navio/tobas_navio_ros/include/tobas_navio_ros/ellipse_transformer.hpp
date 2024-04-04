#pragma once

#include <Eigen/Core>

namespace tobas_navio_ros
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

  bool initialize();

  Eigen::Vector3d transform(const Eigen::Vector3d& mag_raw) const;

  const Eigen::Vector3d& getCenter() const;
  const Eigen::Vector3d& getRadius() const;

  friend std::ostream& operator<<(std::ostream& os, const EllipseTransformer& arg);

private:
  Eigen::Vector3d center_;  // 元の座標系における中心
  Eigen::Vector3d radius_;  // 3軸方向の半径
  Eigen::Matrix3d PSPt_;
};
}  // namespace tobas_navio_ros
