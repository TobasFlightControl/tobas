#include "../include/tobas_dsp/noise_variance_filter.hpp"

using namespace std;

namespace dsp
{
NoiseVarianceFilter::NoiseVarianceFilter()
{
}

void NoiseVarianceFilter::initialize(size_t window_size, double hpf_cutoff_freq, double init_data)
{
  if (window_size <= 0)
    throw runtime_error("Window size must be positive.");

  num_data_ = 1;
  window_size_ = window_size;

  data_buf_.clear();
  data_buf_.push_back(init_data);

  hpf_.initializeFromCutoff(hpf_cutoff_freq, init_data);

  welford_.reset();
  welford_.add(init_data);
}

void NoiseVarianceFilter::update(double data, double dt)
{
  // データをHPFに通す
  hpf_.update(data, dt);

  if (num_data_ < window_size_)  // データが溜まるまではバッファに保存しつつWelfordのアルゴリズムを使う
  {
    welford_.add(hpf_.getOutput());
    data_buf_.push_back(data);
  }
  else  // データ数が時間窓を超えてからは移動分散アルゴリズムを使う
  {
    // データサイズが時間窓と一致した時にWelfordのアルゴリズムから移動分散アルゴリズムに切り替える
    if (num_data_ == window_size_)
    {
      assert(data_buf_.size() == window_size_);
      moving_stat_.initialize(data_buf_);
      data_buf_.clear();
    }

    moving_stat_.add(hpf_.getOutput());
  }

  ++num_data_;
}

double NoiseVarianceFilter::noiseVariance() const
{
  if (num_data_ <= window_size_)
    return welford_.variance();
  else
    return moving_stat_.variance();
}
}  // namespace dsp
