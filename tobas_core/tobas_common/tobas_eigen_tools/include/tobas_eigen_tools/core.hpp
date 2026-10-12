// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <stdexcept>
#include <vector>

#include <eigen3/Eigen/Core>

#include <tobas_math/float.hpp>

namespace tobas
{
namespace eigen
{
/**
 * Create a block diagonal matrix.
 *
 * @param _a Submatrix.
 * @param _num Number of blocks.
 * @return Eigen::MatrixXd
 */
template <typename Derived>
Eigen::MatrixXd blockDiag(const Eigen::MatrixBase<Derived>& _a, Eigen::Index _num)
{
  assert(_num > 0);

  const auto r = _a.rows();
  const auto c = _a.cols();

  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(r * _num, c * _num);
  for (Eigen::Index i = 0; i < _num; ++i) {
    res.block(r * i, c * i, r, c) = _a;
  }

  return res;
}

/**
 * Return a matrix created by repeating `A` in the row or column direction.
 *
 * @param _a Submatrix.
 * @param _num Number of repetitions.
 * @param _axis Repetition direction: `Eigen::Vertical` (rows) or `Eigen::Horizontal` (columns).
 * @return Eigen::MatrixXd
 *
 * @see `numpy.tile()`.
 */
template <typename Derived>
Eigen::MatrixXd tile(const Eigen::MatrixBase<Derived>& _a, Eigen::Index _num, Eigen::DirectionType _axis)
{
  assert(_num > 0);

  const auto r = _a.rows();
  const auto c = _a.cols();

  switch (_axis) {
    case Eigen::Vertical: {
      Eigen::MatrixXd res(r * _num, c);
      for (Eigen::Index i = 0; i < _num; ++i) {
        res.block(r * i, 0, r, c) = _a;
      }
      return res;
    }
    case Eigen::Horizontal: {
      Eigen::MatrixXd res(r, c * _num);
      for (Eigen::Index i = 0; i < _num; ++i) {
        res.block(0, c * i, r, c) = _a;
      }
      return res;
    }
    case Eigen::BothDirections:
    default: {
      throw std::runtime_error("axis must be Eigen::Vertical or Eigen::Horizontal");
    }
  }
}

/**
 * Concatenate two matrices in the row or column direction.
 *
 * @param _a,B Matrices to concatenate.
 * @param _axis Concatenation direction: `Eigen::Vertical` (rows) or `Eigen::Horizontal` (columns).
 * @return Eigen::MatrixXd
 *
 * @see `numpy.concatenate()`.
 */
template <typename T, typename U>
Eigen::MatrixXd concat(const Eigen::MatrixBase<T>& _a, const Eigen::MatrixBase<U>& _b, Eigen::DirectionType _axis)
{
  switch (_axis) {
    case Eigen::Vertical: {
      assert(_a.cols() == _b.cols());
      Eigen::MatrixXd res(_a.rows() + _b.rows(), _a.cols());
      res << _a, _b;
      return res;
    }
    case Eigen::Horizontal: {
      assert(_a.rows() == _b.rows());
      Eigen::MatrixXd res(_a.rows(), _a.cols() + _b.cols());
      res << _a, _b;
      return res;
    }
    case Eigen::BothDirections:
    default: {
      throw std::runtime_error("axis must be Eigen::Vertical or Eigen::Horizontal");
    }
  }
}

/**
 * Concatenate three matrices in the row or column direction.
 *
 * @param _a,B,C Matrices to concatenate.
 * @param _axis Concatenation direction: `Eigen::Vertical` (rows) or `Eigen::Horizontal` (columns).
 * @return Eigen::MatrixXd
 *
 * @see `numpy.concatenate()`.
 */
template <typename T, typename U, typename V>
inline Eigen::MatrixXd concat(
  const Eigen::MatrixBase<T>& _a,
  const Eigen::MatrixBase<U>& _b,
  const Eigen::MatrixBase<V>& _c,
  Eigen::DirectionType _axis)
{
  return concat(concat(_a, _b, _axis), _c, _axis);
}

/** Convert `Eigen::Vector` to `std::vector`. */
template <typename T, int N>
inline std::vector<T> toStdVector(const Eigen::Vector<T, N>& _v)
{
  return std::vector<T>(_v.data(), _v.data() + _v.size());
}

/** Convert `std::vector` to `Eigen::Vector`. */
template <typename T>
inline Eigen::Vector<T, Eigen::Dynamic> fromStdVector(const std::vector<T>& _vec)
{
  return Eigen::Map<const Eigen::Vector<T, Eigen::Dynamic>>(_vec.data(), _vec.size());
}

/** Convert `Eigen::Vector` to `std::array`. */
template <typename T, int N>
inline std::array<T, N> toStdArray(const Eigen::Vector<T, N>& _v)
{
  static_assert(N >= 0);
  std::array<double, N> arr{};
  std::copy_n(_v.data(), N, arr.begin());
  return arr;
}

/** Convert `std::array` to `Eigen::Vector`. */
template <typename T, size_t N>
inline Eigen::Vector<T, N> fromStdArray(const std::array<T, N>& _arr)
{
  static_assert(N >= 0);
  return Eigen::Map<const Eigen::Vector<T, N>>(_arr.data(), N);
}

/** Return true when the matrix is square. */
template <typename Derived>
inline bool isSquare(const Eigen::MatrixBase<Derived>& _a)
{
  return _a.rows() == _a.cols();
}

/** If true, this guarantees that the matrix contains no `nan` or `inf`. */
template <typename Derived>
inline bool isFinite(const Eigen::MatrixBase<Derived>& _x)
{
  return ((_x - _x).array() == (_x - _x).array()).all();
}

/** Return true when two matrices are nearly equal. */
template <typename Derived>
bool isClose(
  const Eigen::MatrixBase<Derived>& _x,
  const Eigen::MatrixBase<Derived>& _y,
  double _abs_tol = 1e-8,
  double _rel_tol = 1e-5)
{
  assert(_x.rows() == _y.rows());
  assert(_x.cols() == _y.cols());

  const auto max_diff = (_x - _y).cwiseAbs().maxCoeff();
  const auto abs_max = _x.cwiseAbs().cwiseMax(_y.cwiseAbs()).maxCoeff();
  return max_diff < _abs_tol || max_diff < _rel_tol * abs_max;
}

/**
 * Symmetrize a square matrix.
 *
 * @tparam Derived
 * @param _a Matrix to symmetrize.
 */
template <typename Derived>
inline void symmetrise(Eigen::MatrixBase<Derived>& _a)
{
  _a = (_a + _a.transpose()) / 2;
}

/** Calculate means along rows. */
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Rows, 1> meanRow(const Eigen::Matrix<Scalar, Rows, Cols>& _a)
{
  Eigen::Matrix<Scalar, Rows, 1> res;
  for (Eigen::Index r = 0; r < Rows; ++r) {
    res(r) = _a.row(r).mean();
  }
  return res;
}

/** Calculate means along columns. */
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Cols, 1> meanCol(const Eigen::Matrix<Scalar, Rows, Cols>& _a)
{
  Eigen::Matrix<Scalar, Cols, 1> res;
  for (Eigen::Index c = 0; c < Cols; ++c) {
    res(c) = _a.col(c).mean();
  }
  return res;
}

/** Calculate variances along rows. */
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Rows, 1> varianceRow(const Eigen::Matrix<Scalar, Rows, Cols>& _a)
{
  const Eigen::Matrix<Scalar, Rows, 1> mean = meanRow(_a);
  Eigen::Matrix<Scalar, Rows, 1> res;
  for (Eigen::Index r = 0; r < Rows; ++r) {
    res(r) = (_a.row(r).array() - mean(r)).square().mean();
  }
  return res;
}

/** Calculate variances along columns. */
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Cols, 1> varianceCol(const Eigen::Matrix<Scalar, Rows, Cols>& _a)
{
  const Eigen::Matrix<Scalar, Cols, 1> mean = meanCol(_a);
  Eigen::Matrix<Scalar, Cols, 1> res;
  for (Eigen::Index c = 0; c < Cols; ++c) {
    res(c) = (_a.col(c).array() - mean(c)).square().mean();
  }
  return res;
}

/** Calculate vector variance. */
template <typename Scalar, int Size>
inline Scalar variance(const Eigen::Vector<Scalar, Size>& _v)
{
  return varianceCol(_v)(0);
}
}  // namespace eigen
}  // namespace tobas
