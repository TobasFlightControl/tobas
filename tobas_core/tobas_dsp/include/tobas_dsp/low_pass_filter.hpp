#pragma once

#include <iostream>

#include "./base_filter.hpp"
#include "./utils.hpp"

namespace dsp
{
template <typename T>
class LowPassFilter : public BaseFilter<T>
{
  using super = BaseFilter<T>;

public:
  explicit LowPassFilter();

  super::error_t update(const T& u, const double& dt) override;

  inline const T& getValue() const override;
  inline void setValue(const T& x) override;

  bool setCutoffFrequency(const double& curoff_freq);

private:
  double T_;
  T y_, prev_u_;
};

template <typename T>
LowPassFilter<T>::LowPassFilter()
{
}

template <typename T>
BaseFilter<T>::error_t LowPassFilter<T>::update(const T& u, const double& dt)
{
  if (dt < 0.)
  {
    y_ = u;
    super::error_code_ = super::E_TIME_STEP_NEGATIVE;
  }
  else if (dt > T_)
  {
    y_ = u;
    super::error_code_ = super::E_TIME_STEP_TOO_LARGE;
  }
  else
  {
    y_ = ((T_ - dt) * y_ + dt * (u + prev_u_)) / (T_ + dt);
    super::error_code_ = super::E_NO_ERROR;
  }

  prev_u_ = u;
  return super::error_code_;
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
  if (cutoff_freq <= 0)
  {
    std::cerr << "Cutoff frequency must be positive." << std::endl;
    return false;
  }

  T_ = 2 * timeConstFromCutoff(cutoff_freq);
  return true;
}
}  // namespace dsp
