#pragma once

#include <cassert>
#include <cmath>

namespace dsp
{
/* 一次遅れフィルタ (ローパスフィルタ)． */
template <typename T>
class LowPassFilter
{
public:
  explicit LowPassFilter();

  void initializeFromTimeConst(const double& time_const, const T& init_state);
  void initializeFromCutoff(const double& cutoff_freq, const T& init_state);

  void update(const T& input_state, const double& sampling_time);

  inline const T& getState() const;

private:
  double time_const_;
  T state_;

  /* カットオフ周波数から一次遅れフィルタの時定数を計算する． */
  inline static double timeConstFromCutoff(const double& cutoff_freq);
};

template <typename T>
LowPassFilter<T>::LowPassFilter()
{
}

template <typename T>
void LowPassFilter<T>::initializeFromTimeConst(const double& time_const, const T& init_state)
{
  assert(time_const >= 0);

  time_const_ = time_const;
  state_ = init_state;
}

template <typename T>
void LowPassFilter<T>::initializeFromCutoff(const double& cutoff_freq, const T& init_state)
{
  initializeFromTimeConst(timeConstFromCutoff(cutoff_freq), init_state);
}

template <typename T>
void LowPassFilter<T>::update(const T& input_state, const double& sampling_time)
{
  assert(sampling_time >= 0);

  const double alpha = time_const_ > 0 ? std::exp(-sampling_time / time_const_) : 0;
  state_ = alpha * state_ + (1 - alpha) * input_state;
}

template <typename T>
inline const T& LowPassFilter<T>::getState() const
{
  return state_;
}

template <typename T>
inline double LowPassFilter<T>::timeConstFromCutoff(const double& cutoff_freq)
{
  assert(cutoff_freq > 0.);
  return 0.5 / M_PI / cutoff_freq;
}
}  // namespace dsp
