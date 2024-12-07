#pragma once

#include <eigen3/Eigen/Core>

template <typename A, typename B>
inline auto operator+(const Eigen::MatrixBase<A>& lhs, const Eigen::DiagonalBase<B>& rhs)
{
  assert(lhs.rows() == rhs.rows());
  assert(lhs.cols() == rhs.cols());

  auto res = lhs.eval();
  res.diagonal() += rhs.diagonal();
  return res;
}

template <typename A, typename B>
inline auto operator-(const Eigen::MatrixBase<A>& lhs, const Eigen::DiagonalBase<B>& rhs)
{
  assert(lhs.rows() == rhs.rows());
  assert(lhs.cols() == rhs.cols());

  auto res = lhs.eval();
  res.diagonal() -= rhs.diagonal();
  return res;
}

template <typename A, typename B>
inline auto operator+(const Eigen::DiagonalBase<B>& lhs, const Eigen::MatrixBase<A>& rhs)
{
  return rhs + lhs;
}

template <typename A, typename B>
inline auto operator-(const Eigen::DiagonalBase<B>& lhs, const Eigen::MatrixBase<A>& rhs)
{
  return -rhs + lhs;
}
