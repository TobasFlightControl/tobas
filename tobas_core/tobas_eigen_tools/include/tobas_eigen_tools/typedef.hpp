#pragma once

#include <eigen3/Eigen/Core>

namespace Eigen
{
using Scalard = Matrix<double, 1, 1>;

using Vector6d = Matrix<double, 6, 1>;
using Matrix6d = Matrix<double, 6, 6>;
using Matrix6Xd = Matrix<double, 6, Dynamic>;
using MatrixX6d = Matrix<double, Dynamic, 6>;

using Diagonal2d = DiagonalMatrix<double, 2>;
using Diagonal3d = DiagonalMatrix<double, 3>;
using Diagonal4d = DiagonalMatrix<double, 4>;
using Diagonal6d = DiagonalMatrix<double, 6>;
using DiagonalXd = DiagonalMatrix<double, Dynamic>;
}  // namespace Eigen
