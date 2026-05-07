// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

namespace Eigen
{
#define PP_CAT2_(a, b) a##b
#define PP_CAT2(a, b) PP_CAT2_(a, b)
#define PP_CAT3(a, b, c) PP_CAT2(PP_CAT2(a, b), c)
#define EIGEN_TYPEDEF(N, ...)                                                                                          \
  using PP_CAT3(Vector, N, d) = Vector<__VA_ARGS__, N>;                                                                \
  using PP_CAT3(Matrix, N, d) = Matrix<__VA_ARGS__, N, N>;                                                             \
  using PP_CAT3(Matrix, N, Xd) = Matrix<__VA_ARGS__, N, Dynamic>;                                                      \
  using PP_CAT3(MatrixX, N, d) = Matrix<__VA_ARGS__, Dynamic, N>;                                                      \
  using PP_CAT3(Diagonal, N, d) = DiagonalMatrix<__VA_ARGS__, N>;

EIGEN_TYPEDEF(2, double);
EIGEN_TYPEDEF(3, double);
EIGEN_TYPEDEF(4, double);
EIGEN_TYPEDEF(5, double);
EIGEN_TYPEDEF(6, double);

#undef EIGEN_TYPEDEF
#undef PP_CAT3
#undef PP_CAT2
#undef PP_CAT2_

// Other types
using Scalard = Matrix<double, 1, 1>;
using DiagonalXd = DiagonalMatrix<double, Dynamic>;
}  // namespace Eigen
