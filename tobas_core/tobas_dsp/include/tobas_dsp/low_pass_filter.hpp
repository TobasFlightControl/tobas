#pragma once

#include <cassert>
#include <cmath>
#include <iostream>

#include "./base_filter.hpp"

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

  bool setCutoffFrequency(const double& fc);

private:
  double wc_;
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

  // Prewarping
  const auto dt_2 = dt / 2.;
  const auto wc = tan(wc_ * dt_2) / dt_2;

  const auto tau = 2. / wc;
  if (dt > tau) {
    y_ = u;
  }
  else {
    y_ = ((tau - dt) * y_ + dt * (u + prev_u_)) / (tau + dt);  // Tustin's method
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
bool LowPassFilter<T>::setCutoffFrequency(const double& fc)
{
  if (fc <= 0.) {
    std::cerr << "The cutoff frequency of low-pass filter must be positive." << std::endl;
    return false;
  }

  wc_ = 2. * M_PI * fc;  // Hz -> rad/s
  return true;
}
}  // namespace dsp
