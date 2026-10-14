// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_eigen_tools/spline.hpp"

namespace tobas::eigen
{
SplineFunction::SplineFunction(const Eigen::VectorXd& _x_vec, const Eigen::VectorXd& _y_vec, size_t _degree)
  : x_min_(_x_vec.minCoeff())
  , x_max_(_x_vec.maxCoeff())
  , spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(
      _y_vec.transpose(),
      std::min<int>(_x_vec.rows() - 1, _degree),
      scaledValues(_x_vec)))
{
  assert(_x_vec.size() >= 2);
  assert(_y_vec.size() >= 2);
  assert(_x_vec.size() == _y_vec.size());
  assert(_degree <= 3);
}

double SplineFunction::operator()(double _x) const
{
  // x values need to be scaled down in extraction as well.
  return spline_(scaledValue(_x))(0);
}

double SplineFunction::scaledValue(double _x) const
{
  return (_x - x_min_) / (x_max_ - x_min_);
}

Eigen::RowVectorXd SplineFunction::scaledValues(const Eigen::VectorXd& _x_vec) const
{
  return _x_vec.unaryExpr([this](double _x) { return scaledValue(_x); }).transpose();
}
}  // namespace tobas::eigen
