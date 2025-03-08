#pragma once

#include "./high_pass_filter.hpp"
#include "./welford.hpp"
#include "./moving_stat.hpp"

namespace dsp
{
/**
 * @brief 信号の高周波ノイズ成分の分散をオンラインで計算する．
 */
template <typename Scalar, int Size, size_t Length>
class NoiseVarianceFilter
{
  using DataType = Eigen::Vector<Scalar, Size>;
  using CovType = Eigen::Matrix<Scalar, Size, Size>;

public:
  explicit NoiseVarianceFilter();

  bool initialize(double hpf_cutoff_freq, const DataType& init_data);
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
bool NoiseVarianceFilter<Scalar, Size, Length>::initialize(double hpf_cutoff_freq, const DataType& init_data)
{
  if (!hpf_.setCutoffFrequency(hpf_cutoff_freq))
    return false;

  hpf_.setValue(init_data);

  num_data_ = 1;
  data_buf_.at(0) = init_data;

  welford_.reset();
  welford_.add(init_data);

  return true;
}

template <typename Scalar, int Size, size_t Length>
void NoiseVarianceFilter<Scalar, Size, Length>::update(const DataType& data, double dt)
{
  // データをHPFに通す
  hpf_.update(data, dt);

  if (num_data_ < Length)  // データが溜まるまではバッファに保存しつつWelfordのアルゴリズムを使う
  {
    welford_.add(hpf_.getValue());
    data_buf_.at(num_data_) = data;
  }
  else  // データ数が時間窓を超えてからは移動分散アルゴリズムを使う
  {
    // データサイズが時間窓と一致した時にWelfordのアルゴリズムから移動分散アルゴリズムに切り替える
    if (num_data_ == Length)
    {
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
  if (num_data_ <= Length)
    return welford_.variance();
  else
    return moving_stat_.variance();
}
}  // namespace dsp
