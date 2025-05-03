#pragma once

#include <iostream>
#include <cassert>

#include "./base_filter.hpp"
#include "./utils.hpp"

namespace dsp
{
template <typename T>
class LowPassFilter : public BaseFilter<T>
{
public:
  explicit LowPassFilter();

  void update(const T& u, const double& dt) override;

  inline const T& getValue() const override;
  inline void setValue(const T& x) override;

  bool setCutoffFrequency(const double& curoff_freq);

private:
  double T_ = 0.;
  T y_, prev_u_;
};

template <typename T>
LowPassFilter<T>::LowPassFilter()
{
}

template <typename T>
void LowPassFilter<T>::update(const T& u, const double& dt)
{
  assert(dt >= 0.);

  if (dt > T_) {
    y_ = u;
  }
  else {
    y_ = ((T_ - dt) * y_ + dt * (u + prev_u_)) / (T_ + dt);
  }

  prev_u_ = u;
}

template <typename T>
inline const T& LowPassFilter<T>::getValue() const
{
  return y_;
}

template <typename T>
inline void LowPassFilter<T>::setValue(const T& x)
{
  y_ = prev_u_ = x;
}

template <typename T>
bool LowPassFilter<T>::setCutoffFrequency(const double& cutoff_freq)
{
  if (cutoff_freq <= 0) {
    std::cerr << "The cutoff frequency of low-pass filter must be positive." << std::endl;
    return false;
  }

  T_ = 2 * timeConstFromCutoff(cutoff_freq);
  return true;
}
}  // namespace dsp
