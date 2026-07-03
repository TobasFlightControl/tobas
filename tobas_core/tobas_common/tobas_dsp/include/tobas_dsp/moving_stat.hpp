// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <deque>

#include <tobas_eigen_tools/core.hpp>

namespace tobas
{
namespace dsp
{
/**
 * @brief Sequential moving mean and moving covariance calculation for vectors (memo: 2-66).
 */
template <typename Scalar, int Size, size_t Length>
class MovingStatistics
{
  using DataType = Eigen::Vector<Scalar, Size>;
  using CovType = Eigen::Matrix<Scalar, Size, Size>;

public:
  explicit MovingStatistics();

  void initialize(const std::array<DataType, Length>& init_data);
  inline void add(const DataType& x);

  inline const DataType& mean() const;
  inline const CovType& variance() const;

private:
  std::deque<DataType> que_;
  DataType m_;  // Mean
  CovType v_;   // Covariance
};

template <typename Scalar, int Size, size_t Length>
MovingStatistics<Scalar, Size, Length>::MovingStatistics()
{
  static_assert(Size > 0);
  static_assert(Length > 0);
}

template <typename Scalar, int Size, size_t Length>
void MovingStatistics<Scalar, Size, Length>::initialize(const std::array<DataType, Length>& init_data)
{
  que_.clear();
  for (const auto& x : init_data) {
    que_.push_back(x);
  }

  DataType data_sum = DataType::Zero();
  for (const auto& x : init_data) {
    data_sum += x;
  }
  m_ = data_sum / Length;

  CovType cov_sum = CovType::Zero();
  for (const auto& x : init_data) {
    const DataType d = x - m_;
    cov_sum += d * d.transpose();
  }
  v_ = cov_sum / Length;
  eigen::symmetrise(v_);
}

template <typename Scalar, int Size, size_t Length>
inline void MovingStatistics<Scalar, Size, Length>::add(const DataType& x_new)
{
  const DataType& x_old = que_.front();

  // Update mean and variance.
  const DataType d = (x_new - x_old) / Length;
  m_ += d;

  const DataType diff_old = x_old - m_;
  const DataType diff_new = x_new - m_;
  v_ += d * d.transpose() + (diff_new * diff_new.transpose() - diff_old * diff_old.transpose()) / Length;
  eigen::symmetrise(v_);

  // Update the queue.
  que_.pop_front();
  que_.push_back(x_new);
}

template <typename Scalar, int Size, size_t Length>
inline const MovingStatistics<Scalar, Size, Length>::DataType& MovingStatistics<Scalar, Size, Length>::mean() const
{
  return m_;
}

template <typename Scalar, int Size, size_t Length>
inline const MovingStatistics<Scalar, Size, Length>::CovType& MovingStatistics<Scalar, Size, Length>::variance() const
{
  return v_;
}
}  // namespace dsp
}  // namespace tobas
