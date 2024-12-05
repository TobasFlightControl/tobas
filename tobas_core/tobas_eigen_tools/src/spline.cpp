#include "../include/tobas_eigen_tools/spline.hpp"

using namespace std;
using namespace Eigen;

namespace eigen
{
SplineFunction::SplineFunction(const VectorXd& x_vec, const VectorXd& y_vec, const size_t& degree)
  : x_min_(x_vec.minCoeff()),
    x_max_(x_vec.maxCoeff()),
    spline_(SplineFitting<Spline<double, 1>>::Interpolate(
      y_vec.transpose(),
      min<int>(x_vec.rows() - 1, degree),
      scaledValues(x_vec)))
{
  assert(x_vec.size() >= 2);
  assert(y_vec.size() >= 2);
  assert(x_vec.size() == y_vec.size());
  assert(degree <= 3);
}

double SplineFunction::operator()(const double& x) const
{
  // x values need to be scaled down in extraction as well.
  return spline_(scaledValue(x))(0);
}

double SplineFunction::scaledValue(const double& x) const
{
  return (x - x_min_) / (x_max_ - x_min_);
}

RowVectorXd SplineFunction::scaledValues(const VectorXd& x_vec) const
{
  return x_vec.unaryExpr([this](const double& x) { return scaledValue(x); }).transpose();
}
}  // namespace eigen
