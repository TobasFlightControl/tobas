#pragma once

#include "./high_pass_filter.hpp"
#include "./welford.hpp"
#include "./moving_stat.hpp"

namespace dsp
{
/* 信号の高周波ノイズ成分の分散をオンラインで計算する． */
class NoiseVarianceFilter
{
public:
  enum error_t : int
  {
    E_NO_ERROR = 0,
    E_HPF_ERROR = -1,
  };

  explicit NoiseVarianceFilter();

  void initialize(size_t window_size, double hpf_cutoff_freq, double init_data);
  error_t update(double data, double dt);

  double noiseVariance() const;

  error_t errorCode() const;
  const char* errorMessage() const;

private:
  error_t error_code_;

  size_t num_data_;
  size_t window_size_;
  std::vector<double> data_buf_;
  HighPassFilter<double> hpf_;
  Welford welford_;
  MovingStatistics moving_stat_;
};
}  // namespace dsp
