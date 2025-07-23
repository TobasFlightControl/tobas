#pragma once

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

#include <tobas_math/core.hpp>

#include "./base_filter.hpp"
#include "./utils.hpp"

namespace dsp
{
/**
 * @brief Second order low-pass filter.
 *
 * @note The sampling frequency should be at least 20 times the cutoff frequency.
 */
template <typename T>
class LowPassFilterP2 : public BaseFilter<T>
{
  static constexpr double kZeta = M_SQRT1_2;  // 共振が発生しない減衰比の最小値

public:
  explicit LowPassFilterP2();

  void update(const T& u, const double& dt) override;

  inline const T& getValue() const override;
  inline void setValue(const T& x) override;

  bool setCutoffFrequency(const double& fc);

private:
  double wn_ = std::numeric_limits<double>::max();  // [rad/s]
  std::array<T, 3> y_;
  std::array<T, 3> u_;

  void shiftHistory();
};

template <typename T>
LowPassFilterP2<T>::LowPassFilterP2()
{
}

template <typename T>
void LowPassFilterP2<T>::update(const T& u, const double& dt)
{
  assert(dt >= 0.);

  const auto wn = prewarp(wn_, dt);
  const auto c = wn * dt;

  if (c >= 2.) {
    setValue(u);
    return;
  }

  shiftHistory();

  const auto c2 = math::sqr(c);
  const auto c2_4 = c2 + 4;
  const auto zc4 = kZeta * c * 4;

  u_[0] = u;
  y_[0] = (c2 * (u_[0] + 2 * u_[1] + u_[2]) + 2 * (4 - c2) * y_[1] - (c2_4 - zc4) * y_[2]) / (c2_4 + zc4);
}

template <typename T>
inline const T& LowPassFilterP2<T>::getValue() const
{
  return y_[0];
}

template <typename T>
inline void LowPassFilterP2<T>::setValue(const T& x)
{
  y_.fill(x);
  u_.fill(x);
}

template <typename T>
bool LowPassFilterP2<T>::setCutoffFrequency(const double& fc)
{
  if (fc <= 0.) {
    std::cerr << "The cutoff frequency of P2 low-pass filter must be positive." << std::endl;
    return false;
  }

  wn_ = 2. * M_PI * fc;
  return true;
}

template <typename T>
void LowPassFilterP2<T>::shiftHistory()
{
  y_[2] = y_[1];
  y_[1] = y_[0];
  u_[2] = u_[1];
  u_[1] = u_[0];
}
}  // namespace dsp
