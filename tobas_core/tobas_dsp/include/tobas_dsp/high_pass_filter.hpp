#pragma once

#include <cassert>
#include <iostream>

#include "./base_filter.hpp"
#include "./utils.hpp"

namespace dsp
{
template <typename T>
class HighPassFilter : public BaseFilter<T>
{
public:
  explicit HighPassFilter();

  void update(const T& u, const double& dt) override;

  inline const T& getValue() const override;
  inline void setValue(const T& x) override;

  bool setCutoffFrequency(const double& curoff_freq);

private:
  double T_ = 0.;
  T y_, prev_u_;
};

template <typename T>
HighPassFilter<T>::HighPassFilter()
{
}

template <typename T>
void HighPassFilter<T>::update(const T& u, const double& dt)
{
  assert(dt >= 0.);

  if (dt > T_) {
    y_ = u;
  }
  else {
    y_ = ((T_ - dt) * y_ + T_ * (u - prev_u_)) / (T_ + dt);
  }

  prev_u_ = u;
}

template <typename T>
inline const T& HighPassFilter<T>::getValue() const
{
  return y_;
}

template <typename T>
inline void HighPassFilter<T>::setValue(const T& x)
{
  y_ = prev_u_ = x;
}

template <typename T>
bool HighPassFilter<T>::setCutoffFrequency(const double& cutoff_freq)
{
  if (cutoff_freq <= 0) {
    std::cerr << "The cutoff frequency of high-pass filter must be positive." << std::endl;
    return false;
  }

  T_ = 2 * timeConstFromCutoff(cutoff_freq);
  return true;
}
}  // namespace dsp
