#pragma once

#include <eigen3/unsupported/Eigen/CXX11/Tensor>

namespace Eigen
{
using Tensor1Xd = Tensor<double, 1>;
using Tensor2Xd = Tensor<double, 2>;
using Tensor3Xd = Tensor<double, 3>;
using Tensor4Xd = Tensor<double, 4>;

MatrixXd operator*(const RowVectorXd& lhs, const Tensor3Xd& rhs);
}  // namespace Eigen
