#pragma once

#include <cassert>
#include <cmath>

#include "./assert.hpp"

namespace tobas_std
{
/* 一次遅れフィルタ (ローパスフィルタ)． */
template <typename T>
class FirstOrderFilter
{
public:
  explicit FirstOrderFilter();

  void initializeFromTimeConst(const double& time_const, const T& init_state);
  void initializeFromCutoff(const double& cutoff_freq, const T& init_state);

  void update(const T& input_state, const double& sampling_time);

  inline const T& getState() const;
  inline const bool& isInitialized() const;

private:
  bool is_initialized_ = false;
  double time_const_;
  T state_;

  /* カットオフ周波数から一次遅れフィルタの時定数を計算する． */
  inline static double timeConstFromCutoff(const double& cutoff_freq);
};

template <typename T>
FirstOrderFilter<T>::FirstOrderFilter()
{
}

template <typename T>
void FirstOrderFilter<T>::initializeFromTimeConst(const double& time_const, const T& init_state)
{
  assert(time_const >= 0);

  time_const_ = time_const;
  state_ = init_state;

  is_initialized_ = true;
}

template <typename T>
void FirstOrderFilter<T>::initializeFromCutoff(const double& cutoff_freq, const T& init_state)
{
  initializeFromTimeConst(timeConstFromCutoff(cutoff_freq), init_state);
}

template <typename T>
void FirstOrderFilter<T>::update(const T& input_state, const double& sampling_time)
{
  assert(is_initialized_);
  assertWithMsg(sampling_time >= 0, "Invalid sampling time: " << sampling_time);

  const double alpha = time_const_ > 0 ? std::exp(-sampling_time / time_const_) : 0;
  state_ = alpha * state_ + (1 - alpha) * input_state;
}

template <typename T>
inline const T& FirstOrderFilter<T>::getState() const
{
  return state_;
}

template <typename T>
inline const bool& FirstOrderFilter<T>::isInitialized() const
{
  return is_initialized_;
}

template <typename T>
inline double FirstOrderFilter<T>::timeConstFromCutoff(const double& cutoff_freq)
{
  assert(cutoff_freq > 0.);
  return 0.5 / M_PI / cutoff_freq;
}
}  // namespace tobas_std
