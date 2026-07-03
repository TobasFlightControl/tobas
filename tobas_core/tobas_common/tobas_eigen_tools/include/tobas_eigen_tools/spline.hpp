// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/Splines>

namespace tobas
{
namespace eigen
{
/**
 * @brief Polynomial interpolation class that accepts arbitrary horizontal-axis values.
 * https://stackoverflow.com/questions/29822041/
 *
 * @note Memory usage may become huge when the number of data points is too large.
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
}  // namespace eigen
}  // namespace tobas
