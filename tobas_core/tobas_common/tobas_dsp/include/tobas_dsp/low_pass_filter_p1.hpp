#pragma once

#include <cassert>
#include <cmath>
#include <iostream>

#include <tobas_math/definitions.hpp>

#include "./base_filter.hpp"
#include "./utils.hpp"

namespace dsp
{
/**
 * @brief First order low-pass filter.
 *
 * @note The sampling frequency should be at least 10 times the cutoff frequency.
 */
template <typename T>
class LowPassFilterP1 : public BaseFilter<T>
{
public:
  explicit LowPassFilterP1();

  void update(const T& u, const double& dt) override;

  inline const T& getValue() const override;
  inline void setValue(const T& x) override;

  bool setCutoffFrequency(const double& fc_hz);

private:
  double wc_ = std::numeric_limits<double>::max();  // [rad/s]
  T y_, prev_u_;
};

template <typename T>
LowPassFilterP1<T>::LowPassFilterP1()
{
}

template <typename T>
void LowPassFilterP1<T>::update(const T& u, const double& dt)
{
  assert(dt >= 0.);

  const auto wc = prewarp(wc_, dt);
  const auto tau = 2. / wc;

  y_ = dt < tau ? ((tau - dt) * y_ + dt * (u + prev_u_)) / (tau + dt) : u;
  prev_u_ = u;
}

template <typename T>
inline const T& LowPassFilterP1<T>::getValue() const
{
  return y_;
}

template <typename T>
inline void LowPassFilterP1<T>::setValue(const T& x)
{
  y_ = prev_u_ = x;
}

template <typename T>
bool LowPassFilterP1<T>::setCutoffFrequency(const double& fc_hz)
{
  if (fc_hz <= 0.) {
    std::cerr << "The cutoff frequency of P1 low-pass filter must be positive." << std::endl;
    return false;
  }

  wc_ = M_2PI * fc_hz;  // Hz -> rad/s
  return true;
}
}  // namespace dsp
