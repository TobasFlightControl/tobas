#pragma once

#include <cmath>
#include <cassert>
#include <stdexcept>

#include "./utils.hpp"

namespace dsp
{
template <typename T>
class LowPassFilter
{
public:
  explicit LowPassFilter();

  void initialize(const double& cutoff_freq, const T& init_value);
  void update(const T& u, const double& dt);

  inline const T& getOutput() const;

private:
  double T_;
  T y_, prev_u_;
};

template <typename T>
LowPassFilter<T>::LowPassFilter()
{
}

template <typename T>
void LowPassFilter<T>::initialize(const double& cutoff_freq, const T& init_value)
{
  if (cutoff_freq <= 0)
    throw std::runtime_error("Cutoff frequency must be positive.");

  T_ = 2 * timeConstFromCutoff(cutoff_freq);
  y_ = prev_u_ = init_value;
}

template <typename T>
void LowPassFilter<T>::update(const T& u, const double& dt)
{
  assert(0 <= dt);
  assert(dt <= T_);

  y_ = ((T_ - dt) * y_ + dt * (u + prev_u_)) / (T_ + dt);
  prev_u_ = u;
}

template <typename T>
inline const T& LowPassFilter<T>::getOutput() const
{
  return y_;
}
}  // namespace dsp
