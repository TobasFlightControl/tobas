#pragma once

#include <cassert>
#include <cmath>
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

  bool setCutoffFrequency(const double& fc);

private:
  double wc_ = NAN;  // [rad/s]
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

  const auto wc = prewarp(wc_, dt);
  const auto tau = 2. / wc;

  y_ = dt < tau ? ((tau - dt) * y_ + tau * (u - prev_u_)) / (tau + dt) : u;
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
bool HighPassFilter<T>::setCutoffFrequency(const double& fc)
{
  if (fc <= 0.) {
    std::cerr << "The cutoff frequency of high-pass filter must be positive." << std::endl;
    return false;
  }

  wc_ = 2. * M_PI * fc;  // Hz -> rad/s
  return true;
}
}  // namespace dsp
