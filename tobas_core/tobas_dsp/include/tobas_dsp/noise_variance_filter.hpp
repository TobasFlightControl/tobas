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
  explicit NoiseVarianceFilter();

  void initialize(size_t window_size, double hpf_cutoff_freq, double init_data);

  void update(double data, double dt);

  double noiseVariance() const;

private:
  size_t num_data_;
  size_t window_size_;
  std::vector<double> data_buf_;
  HighPassFilter<double> hpf_;
  Welford welford_;
  MovingStatistics moving_stat_;
};
}  // namespace dsp
