#pragma once

#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

namespace eigen_tools
{
/**
 * @brief 任意の横軸値を引数にとれる多項式補完クラス．
 * https://stackoverflow.com/questions/29822041/
 *
 * @note データ数が多すぎるとメモリ消費量が膨大になるらしい．
 */
class SplineFunction
{
public:
  explicit SplineFunction(const Eigen::VectorXd& x_vec, const Eigen::VectorXd& y_vec, const size_t& degree);

  double operator()(const double& x) const;

private:
  double x_min_;
  double x_max_;
  Eigen::Spline<double, 1> spline_;  // Spline of one-dimensional points

  /* Helpers to scale X values down to [0, 1]. */
  double scaledValue(const double& x) const;
  Eigen::RowVectorXd scaledValues(const Eigen::VectorXd& x_vec) const;
};
}  // namespace eigen_tools
