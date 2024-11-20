#pragma once

#include <cmath>
#include <array>
#include <iostream>

#include <tobas_math/core.hpp>

#include "./base_filter.hpp"

namespace dsp
{
/* cf. https://qiita.com/yknk0104/items/9519fa02dfd37ea4b228 */
template <typename T>
class NotchFilter : public BaseFilter<T>
{
  using super = BaseFilter<T>;

public:
  explicit NotchFilter();

  super::error_t update(const T& u, const double& dt) override;

  inline const T& getValue() const override;
  inline void setValue(const T& x) override;

  bool setCenterFrequency(const double& fn_hz);
  bool setBandwidth(const double& bw_hz);
  bool setDepth(const double& depth);

private:
  double wn_ = std::numeric_limits<double>::max();  // [rad/s]
  double bw_ = 0.;                                  // [rad/s]
  double d_ = 0.;                                   // [-]

  std::array<T, 3> y_;
  std::array<T, 3> u_;

  void shiftHistory();
};

template <typename T>
NotchFilter<T>::NotchFilter()
{
}

template <typename T>
BaseFilter<T>::error_t NotchFilter<T>::update(const T& u, const double& dt)
{
  if (dt < 0.)
    return super::error_code_ = super::E_TIME_STEP_NEGATIVE;

  shiftHistory();

  if (bw_ == 0.)
  {
    u_[0] = y_[0] = u;
    return error_code_ = super::E_NO_ERROR;
  }

  const auto q = wn_ / bw_;
  const auto c = dt * wn_ / 2;
  const auto c2 = math::sqr(c);
  const auto c2_plus_1 = c2 + 1;
  const auto c2_minus_1 = c2 - 1;
  const auto cd_qinv = c * d / q;
  const auto c1_qinv = c / q;

  const auto n0 = c2_plus_1 + cd_qinv;
  const auto n1 = 2 * c2_minus_1;
  const auto n2 = c2_plus_1 - cd_qinv;
  const auto d0 = c2_plus_1 + c1_qinv;
  const auto d1 = 2 * c2_minus_1;
  const auto d2 = c2_plus_1 - c1_qinv;

  u_[0] = u;
  y_[0] = (n0 * u_[0] + n1 * u_[1] + n2 * u_[2] - d1 * y[1] - d2 * y[2]) / d0;

  return error_code_ = super::E_NO_ERROR;
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
bool NotchFilter<T>::setBandwidth(const double& bw_hz)
{
  if (bw_hz < 0.)
  {
    std::cerr << "The bandwidth of notch filter must be non-negative." << std::endl;
    return false;
  }

  bw_ = (2 * M_PI) * bw_hz;
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
