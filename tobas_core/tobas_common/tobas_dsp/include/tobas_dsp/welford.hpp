// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_eigen_tools/core.hpp>

namespace tobas
{
namespace dsp
{
/**
 * @brief Sequential mean and variance calculation using Welford's algorithm (memo: 2-65).
 *
 * @note Numerical error is small, but it stores the product of variance and sample count.
 * It diverges if the sample count becomes too large.
 */
template <typename Scalar, int Size>
class Welford
{
  using DataType = Eigen::Vector<Scalar, Size>;
  using CovType = Eigen::Matrix<Scalar, Size, Size>;

public:
  explicit Welford();

  /* Reset internal variables. */
  void reset();

  /* Add new data. */
  inline void add(const DataType& x);

  /* Get the mean. */
  inline const DataType& mean() const;

  /* Get the variance. */
  inline CovType variance() const;

  /* Get the number of samples. */
  inline size_t count() const;

private:
  size_t n_;       // Number of samples.
  DataType mean_;  // Mean.
  CovType var_n_;  // Product of variance and sample count.
};

template <typename Scalar, int Size>
Welford<Scalar, Size>::Welford()
{
  static_assert(Size > 0);
  reset();
}

template <typename Scalar, int Size>
void Welford<Scalar, Size>::reset()
{
  n_ = 0;
  mean_.setZero();
  var_n_.setZero();
}

template <typename Scalar, int Size>
inline void Welford<Scalar, Size>::add(const DataType& x)
{
  ++n_;

  const DataType d = x - mean_;
  mean_ += d / n_;

  const DataType d2 = x - mean_;
  var_n_ += d * d2.transpose();
  eigen::symmetrise(var_n_);
}

template <typename Scalar, int Size>
inline const Welford<Scalar, Size>::DataType& Welford<Scalar, Size>::mean() const
{
  return mean_;
}

template <typename Scalar, int Size>
inline Welford<Scalar, Size>::CovType Welford<Scalar, Size>::variance() const
{
  if (n_ > 0) {
    return var_n_ / n_;
  }
  else {
    return CovType::Zero();
  }
}

template <typename Scalar, int Size>
inline size_t Welford<Scalar, Size>::count() const
{
  return n_;
}
}  // namespace dsp
}  // namespace tobas
