// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <vector>

#include <eigen3/Eigen/Core>

#include <tobas_math/float.hpp>

namespace tobas
{
namespace eigen
{
/**
 * @brief Create a block diagonal matrix.
 *
 * @param A Submatrix.
 * @param num Number of blocks.
 *
 * @return Eigen::MatrixXd
 */
template <typename Derived>
Eigen::MatrixXd blockDiag(const Eigen::MatrixBase<Derived>& A, const Eigen::Index& num)
{
  assert(num > 0);

  const auto r = A.rows();
  const auto c = A.cols();

  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(r * num, c * num);
  for (Eigen::Index i = 0; i < num; ++i) {
    res.block(r * i, c * i, r, c) = A;
  }

  return res;
}

/**
 * @brief Return a matrix created by repeating `A` in the row or column direction.
 * cf. `numpy.tile()`.
 *
 * @param A Submatrix.
 * @param num Number of repetitions.
 * @param axis Repetition direction: Row (0) or Column (1).
 *
 * @return Eigen::MatrixXd
 */
template <typename Derived>
Eigen::MatrixXd tile(const Eigen::MatrixBase<Derived>& A, const Eigen::Index& num, const uint8_t& axis)
{
  assert(num > 0);

  const auto r = A.rows();
  const auto c = A.cols();

  switch (axis) {
    case 0: {
      Eigen::MatrixXd res(r * num, c);
      for (Eigen::Index i = 0; i < num; ++i) {
        res.block(r * i, 0, r, c) = A;
      }
      return res;
    }
    case 1: {
      Eigen::MatrixXd res(r, c * num);
      for (Eigen::Index i = 0; i < num; ++i) {
        res.block(0, c * i, r, c) = A;
      }
      return res;
    }
    default: {
      throw std::runtime_error("axis must be 0 or 1");
    }
  }
}

/**
 * @brief Concatenate two matrices in the row or column direction.
 * cf. `numpy.concatenate()`.
 *
 * @param A,B Matrices to concatenate.
 * @param axis Concatenation direction: Row (0) or Column (1).
 *
 * @return Eigen::MatrixXd
 */
template <typename T, typename U>
Eigen::MatrixXd concat(const Eigen::MatrixBase<T>& A, const Eigen::MatrixBase<U>& B, const uint8_t& axis)
{
  switch (axis) {
    case 0: {
      assert(A.cols() == B.cols());
      Eigen::MatrixXd res(A.rows() + B.rows(), A.cols());
      res << A, B;
      return res;
    }
    case 1: {
      assert(A.rows() == B.rows());
      Eigen::MatrixXd res(A.rows(), A.cols() + B.cols());
      res << A, B;
      return res;
    }
    default: {
      throw std::runtime_error("axis must be 0 or 1");
    }
  }
}

/**
 * @brief Concatenate three matrices in the row or column direction.
 * cf. `numpy.concatenate()`.
 *
 * @param A,B,C Matrices to concatenate.
 * @param axis Concatenation direction: Row (0) or Column (1).
 *
 * @return Eigen::MatrixXd
 */
template <typename T, typename U, typename V>
inline Eigen::MatrixXd
concat(const Eigen::MatrixBase<T>& A, const Eigen::MatrixBase<U>& B, const Eigen::MatrixBase<V>& C, const uint8_t& axis)
{
  return concat(concat(A, B, axis), C, axis);
}

/* Convert `Eigen::Vector` to `std::vector`. */
template <typename T, int N>
inline std::vector<T> toStdVector(const Eigen::Vector<T, N>& v)
{
  return std::vector<T>(v.data(), v.data() + v.size());
}

/* Convert `std::vector` to `Eigen::Vector`. */
template <typename T>
inline Eigen::Vector<T, Eigen::Dynamic> fromStdVector(const std::vector<T>& vec)
{
  return Eigen::Map<const Eigen::Vector<T, Eigen::Dynamic>>(vec.data(), vec.size());
}

/* Convert `Eigen::Vector` to `std::array`. */
template <typename T, int N>
inline std::array<T, N> toStdArray(const Eigen::Vector<T, N>& v)
{
  static_assert(N >= 0);
  std::array<double, N> arr{};
  std::copy_n(v.data(), N, arr.begin());
  return arr;
}

/* Convert `std::array` to `Eigen::Vector`. */
template <typename T, size_t N>
inline Eigen::Vector<T, N> fromStdArray(const std::array<T, N>& arr)
{
  static_assert(N >= 0);
  return Eigen::Map<const Eigen::Vector<T, N>>(arr.data(), N);
}

/* Return true when the matrix is square. */
template <typename Derived>
inline bool isSquare(const Eigen::MatrixBase<Derived>& A)
{
  return A.rows() == A.cols();
}

/* If true, this guarantees that the matrix contains no `nan` or `inf`. */
template <typename Derived>
inline bool isFinite(const Eigen::MatrixBase<Derived>& x)
{
  return ((x - x).array() == (x - x).array()).all();
}

/* Return true when two matrices are nearly equal. */
template <typename Derived>
bool isClose(
  const Eigen::MatrixBase<Derived>& x,
  const Eigen::MatrixBase<Derived>& y,
  const double& abs_tol = 1e-8,
  const double& rel_tol = 1e-5)
{
  assert(x.rows() == y.rows());
  assert(x.cols() == y.cols());

  const auto max_diff = (x - y).cwiseAbs().maxCoeff();
  const auto abs_max = x.cwiseAbs().cwiseMax(y.cwiseAbs()).maxCoeff();
  return max_diff < abs_tol || max_diff < rel_tol * abs_max;
}

/**
 * @brief Symmetrize a square matrix.
 *
 * @tparam Derived
 * @param A Matrix to symmetrize.
 */
template <typename Derived>
inline void symmetrise(Eigen::MatrixBase<Derived>& A)
{
  A = (A + A.transpose()) / 2;
}

/* Calculate means along rows. */
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Rows, 1> meanRow(const Eigen::Matrix<Scalar, Rows, Cols>& A)
{
  Eigen::Matrix<Scalar, Rows, 1> res;
  for (Eigen::Index r = 0; r < Rows; ++r) {
    res(r) = A.row(r).mean();
  }
  return res;
}

/* Calculate means along columns. */
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Cols, 1> meanCol(const Eigen::Matrix<Scalar, Rows, Cols>& A)
{
  Eigen::Matrix<Scalar, Cols, 1> res;
  for (Eigen::Index c = 0; c < Cols; ++c) {
    res(c) = A.col(c).mean();
  }
  return res;
}

/* Calculate variances along rows. */
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Rows, 1> varianceRow(const Eigen::Matrix<Scalar, Rows, Cols>& A)
{
  const Eigen::Matrix<Scalar, Rows, 1> mean = meanRow(A);
  Eigen::Matrix<Scalar, Rows, 1> res;
  for (Eigen::Index r = 0; r < Rows; ++r) {
    res(r) = (A.row(r).array() - mean(r)).square().mean();
  }
  return res;
}

/* Calculate variances along columns. */
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Cols, 1> varianceCol(const Eigen::Matrix<Scalar, Rows, Cols>& A)
{
  const Eigen::Matrix<Scalar, Cols, 1> mean = meanCol(A);
  Eigen::Matrix<Scalar, Cols, 1> res;
  for (Eigen::Index c = 0; c < Cols; ++c) {
    res(c) = (A.col(c).array() - mean(c)).square().mean();
  }
  return res;
}

/* Calculate vector variance. */
template <typename Scalar, int Size>
inline Scalar variance(const Eigen::Vector<Scalar, Size>& v)
{
  return varianceCol(v)(0);
}
}  // namespace eigen
}  // namespace tobas
