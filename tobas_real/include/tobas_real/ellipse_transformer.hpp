#pragma once

#include <Eigen/Core>

namespace tobas_real
{
/**
 * @brief 任意の楕円体 (x^T A x + b^T x + 1 = 0) を原点中心の位球に射影する．
 * https://rikei-tawamure.com/entry/2021/09/27/111205
 */
class EllipseTransformer
{
public:
  float a_xx;
  float a_yy;
  float a_zz;
  float a_xy;
  float a_yz;
  float a_zx;
  float b_x;
  float b_y;
  float b_z;
  float c;

  explicit EllipseTransformer();

  void initialize();
  Eigen::Vector3f transform(const Eigen::Vector3f& mag_raw);

  friend std::ostream& operator<<(std::ostream& os, const EllipseTransformer& arg);

private:
  Eigen::Matrix3f A_;
  Eigen::Vector3f b_;
};
}  // namespace tobas_real
