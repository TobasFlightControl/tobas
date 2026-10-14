// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

template <typename A, typename B>
inline auto operator+(const Eigen::MatrixBase<A>& _lhs, const Eigen::DiagonalBase<B>& _rhs)
{
  assert(_lhs.rows() == _rhs.rows());
  assert(_lhs.cols() == _rhs.cols());

  auto res = _lhs.eval();
  res.diagonal() += _rhs.diagonal();
  return res;
}

template <typename A, typename B>
inline auto operator-(const Eigen::MatrixBase<A>& _lhs, const Eigen::DiagonalBase<B>& _rhs)
{
  assert(_lhs.rows() == _rhs.rows());
  assert(_lhs.cols() == _rhs.cols());

  auto res = _lhs.eval();
  res.diagonal() -= _rhs.diagonal();
  return res;
}

template <typename A, typename B>
inline auto operator+(const Eigen::DiagonalBase<B>& _lhs, const Eigen::MatrixBase<A>& _rhs)
{
  return _rhs + _lhs;
}

template <typename A, typename B>
inline auto operator-(const Eigen::DiagonalBase<B>& _lhs, const Eigen::MatrixBase<A>& _rhs)
{
  return -_rhs + _lhs;
}
