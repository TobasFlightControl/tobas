#pragma once

#include <cmath>
#include <array>
#include <iostream>
#include <cassert>

#include <tobas_math/core.hpp>

#include "./base_filter.hpp"

namespace dsp
{
/* cf. https://qiita.com/yknk0104/items/9519fa02dfd37ea4b228 */
template <typename T>
class NotchFilter : public BaseFilter<T>
{
  static constexpr size_t kHistorySize = 3;

public:
  explicit NotchFilter();

  void update(const T& u, const double& dt) override;
  void bypass(const T& u);

  inline const T& getValue() const override;
  inline void setValue(const T& x) override;

  bool setCenterFrequency(const double& fn_hz);
  bool setQValue(const double& q);
  bool setDepth(const double& depth);

private:
  double wn_ = NAN;  // [rad/s]
  double q_ = 0.;    // [-]
  double d_ = 0.;    // [-]

  std::array<T, kHistorySize> y_;
  std::array<T, kHistorySize> u_;

  void shiftHistory();
};

template <typename T>
NotchFilter<T>::NotchFilter()
{
}

template <typename T>
void NotchFilter<T>::update(const T& u, const double& dt)
{
  assert(dt >= 0.);

  if (dt >= 2 / wn_)
  {
    for (size_t i = 0; i < kHistorySize; ++i)
    {
      y_[i] = u;
      u[i] = u;
    }
    return;
  }

  shiftHistory();

  const auto dt_2 = dt / 2;
  const auto wn = tan(wn_ * dt_2) / dt_2;  // Prewarping
  const auto c = wn * dt_2;
  const auto c2 = math::sqr(c);
  const auto c2_plus_1 = c2 + 1;
  const auto c2_minus_1 = c2 - 1;
  const auto c1_qinv = c / q_;
  const auto cd_qinv = c1_qinv * d_;

  const auto n0 = c2_plus_1 + cd_qinv;
  const auto n1 = 2 * c2_minus_1;
  const auto n2 = c2_plus_1 - cd_qinv;
  const auto d0 = c2_plus_1 + c1_qinv;
  const auto d1 = 2 * c2_minus_1;
  const auto d2 = c2_plus_1 - c1_qinv;

  u_[0] = u;
  y_[0] = (n0 * u_[0] + n1 * u_[1] + n2 * u_[2] - d1 * y_[1] - d2 * y_[2]) / d0;
}

template <typename T>
void NotchFilter<T>::bypass(const T& u)
{
  shiftHistory();
  u_[0] = y_[0] = u;
}

template <typename T>
inline const T& NotchFilter<T>::getValue() const
{
  return y_[0];
}

template <typename T>
inline void NotchFilter<T>::setValue(const T& x)
{
  y_.fill(x);
  u_.fill(x);
}

template <typename T>
bool NotchFilter<T>::setCenterFrequency(const double& fn_hz)
{
  if (fn_hz <= 0.)
  {
    std::cerr << "The center frequency of notch filter must be positive." << std::endl;
    return false;
  }

  wn_ = (2 * M_PI) * fn_hz;
  return true;
}

template <typename T>
bool NotchFilter<T>::setQValue(const double& q)
{
  if (q <= 0.)
  {
    std::cerr << "The Q value of notch filter must be positive." << std::endl;
    return false;
  }

  q_ = q;
  return true;
}

template <typename T>
bool NotchFilter<T>::setDepth(const double& depth)
{
  if (depth < 0. || 1. < depth)
  {
    std::cerr << "The depth of notch filter must be in range of (0,1)." << std::endl;
    return false;
  }

  d_ = depth;
  return true;
}

template <typename T>
void NotchFilter<T>::shiftHistory()
{
  y_[2] = y_[1];
  y_[1] = y_[0];
  u_[2] = u_[1];
  u_[1] = u_[0];
}
}  // namespace dsp
