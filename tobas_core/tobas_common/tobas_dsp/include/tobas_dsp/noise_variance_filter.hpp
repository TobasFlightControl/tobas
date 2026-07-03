// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./high_pass_filter.hpp"
#include "./moving_stat.hpp"
#include "./welford.hpp"

namespace tobas
{
namespace dsp
{
/**
 * @brief Online variance calculation for high-frequency noise components in a signal.
 */
template <typename Scalar, int Size, size_t Length>
class NoiseVarianceFilter
{
  using DataType = Eigen::Vector<Scalar, Size>;
  using CovType = Eigen::Matrix<Scalar, Size, Size>;

public:
  explicit NoiseVarianceFilter();

  void initialize(double hpf_cutoff_freq, const DataType& init_data);
  void update(const DataType& data, double dt);

  inline CovType noiseVariance() const;

private:
  size_t num_data_;
  std::array<DataType, Length> data_buf_;
  HighPassFilter<DataType> hpf_;
  Welford<Scalar, Size> welford_;
  MovingStatistics<Scalar, Size, Length> moving_stat_;
};

template <typename Scalar, int Size, size_t Length>
NoiseVarianceFilter<Scalar, Size, Length>::NoiseVarianceFilter()
{
}

template <typename Scalar, int Size, size_t Length>
void NoiseVarianceFilter<Scalar, Size, Length>::initialize(double hpf_cutoff_freq, const DataType& init_data)
{
  hpf_.setCutoffFrequency(hpf_cutoff_freq);
  hpf_.setValue(init_data);

  num_data_ = 1;
  data_buf_.front() = init_data;

  welford_.reset();
  welford_.add(init_data);
}

template <typename Scalar, int Size, size_t Length>
void NoiseVarianceFilter<Scalar, Size, Length>::update(const DataType& data, double dt)
{
  // Pass data through the HPF.
  hpf_.update(data, dt);

  if (num_data_ < Length)  // Use Welford's algorithm while buffering data until the window is full.
  {
    welford_.add(hpf_.getValue());
    data_buf_.at(num_data_) = data;
  }
  else  // Use the moving variance algorithm after the number of samples exceeds the time window.
  {
    // Switch from Welford's algorithm to the moving variance algorithm when the data size reaches the time window.
    if (num_data_ == Length) {
      assert(data_buf_.size() == Length);
      moving_stat_.initialize(data_buf_);
    }

    moving_stat_.add(hpf_.getValue());
  }

  ++num_data_;
}

template <typename Scalar, int Size, size_t Length>
inline NoiseVarianceFilter<Scalar, Size, Length>::CovType
NoiseVarianceFilter<Scalar, Size, Length>::noiseVariance() const
{
  if (num_data_ <= Length) {
    return welford_.variance();
  }
  else {
    return moving_stat_.variance();
  }
}
}  // namespace dsp
}  // namespace tobas
