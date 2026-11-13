// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <vector>

#include <eigen3/Eigen/Core>

namespace tobas_kdl_msgs
{
inline void vectorXdEigenToStd(const Eigen::VectorXd& _e, std::vector<double>& _s)
{
  _s.resize(_e.size());
  std::copy(_e.begin(), _e.end(), _s.begin());
}

inline void vectorXdStdToEigen(const std::vector<double>& _s, Eigen::VectorXd& _e)
{
  _e.conservativeResize(_s.size());
  std::copy(_s.begin(), _s.end(), _e.data());
}

inline void matrix3dEigenToStd(const Eigen::Matrix3d& _e, std::array<double, 9>& _s)
{
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      _s[r * 3 + c] = _e(r, c);
    }
  }
}

inline void matrix3dStdToEigen(const std::array<double, 9>& _s, Eigen::Matrix3d& _e)
{
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      _e(r, c) = _s[r * 3 + c];
    }
  }
}
}  // namespace tobas_kdl_msgs
