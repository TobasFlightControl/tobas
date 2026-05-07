// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <functional>

#include <eigen3/Eigen/Core>

namespace std
{
/**
 * @brief Hash function for Eigen matrix and vector.
 * https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
 */
template <typename Scalar, int Rows, int Cols>
struct hash<Eigen::Matrix<Scalar, Rows, Cols>>
{
  size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      const auto elem = *(matrix.data() + i);
      seed ^= std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};
}  // namespace std
