#pragma once

#include <vector>
#include <array>
#include <eigen3/Eigen/Core>

namespace tobas_kdl_msgs
{
inline void vectorXdEigenToStd(const Eigen::VectorXd& e, std::vector<double>& s)
{
  s.resize(e.size());
  std::copy(e.begin(), e.end(), s.begin());
}

inline void vectorXdStdToEigen(const std::vector<double>& s, Eigen::VectorXd& e)
{
  e.conservativeResize(s.size());
  std::copy(s.begin(), s.end(), e.data());
}

inline void matrix3dEigenToStd(const Eigen::Matrix3d& e, std::array<double, 9>& s)
{
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      s[r * 3 + c] = e(r, c);
    }
  }
}

inline void matrix3dStdToEigen(const std::array<double, 9>& s, Eigen::Matrix3d& e)
{
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      e(r, c) = s[r * 3 + c];
    }
  }
}
}  // namespace tobas_kdl_msgs
