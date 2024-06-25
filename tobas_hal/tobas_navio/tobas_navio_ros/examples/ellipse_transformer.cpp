#include <iostream>

#include <tobas_math/core.hpp>

#include <tobas_navio_ros/ellipse_transformer.hpp>

using namespace std;
using namespace Eigen;

int main()
{
  // 変換元の楕円体を定義
  const double rx = 2, ry = 1, rz = 1;  // 半径
  const double x0 = 2, y0 = 1, z0 = 1;  // 中心

  const double rx2 = math::sqr(rx);
  const double ry2 = math::sqr(ry);
  const double rz2 = math::sqr(rz);

  tobas_navio_ros::EllipseTransformer trans;
  trans.a_xx = 1 / rx2;
  trans.a_yy = 1 / ry2;
  trans.a_zz = 1 / rz2;
  trans.b_x = -2 * x0 / rx2;
  trans.b_y = -2 * y0 / ry2;
  trans.b_z = -2 * z0 / rz2;
  trans.c = math::sqr(x0) / rx2 + math::sqr(y0) / ry2 + math::sqr(z0) / rz2 - 1;

  trans.initialize();

  // 中心と半径を表示
  cout << "Center:" << trans.getCenter().transpose() << endl;
  cout << "Radius:" << trans.getRadius().transpose() << endl;

  // 楕円体上の点を単位球上の点に写像
  const Vector3d x_raw(4, 1, 1);
  const Vector3d x_unit = trans.transform(x_raw);
  cout << "Raw vector: " << x_raw.transpose() << endl;
  cout << "Transformed vector: " << x_unit.transpose() << endl;
}
