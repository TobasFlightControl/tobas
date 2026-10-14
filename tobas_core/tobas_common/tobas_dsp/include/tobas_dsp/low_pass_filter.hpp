// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cassert>
#include <cmath>

#include <tobas_math/definitions.hpp>

#include "./base_filter.hpp"
#include "./utils.hpp"

namespace tobas::dsp
{
/**
 * First order low-pass filter.
 *
 * @note The sampling frequency should be at least 10 times the cutoff frequency.
 */
template <typename T>
class LowPassFilter : public BaseFilter<T>
{
public:
  explicit LowPassFilter();

  void update(const T& _u, double _dt) override;

  inline const T& getValue() const override;
  inline void setValue(const T& _x) override;

  void setCutoffFrequency(double _fc_hz);

private:
  double wc_ = INFINITY;  ///< [rad/s]
  T y_{};
  T prev_u_{};
};

template <typename T>
LowPassFilter<T>::LowPassFilter()
{
}

template <typename T>
void LowPassFilter<T>::update(const T& _u, double _dt)
{
  assert(_dt >= 0.0);

  const auto wc = prewarp(wc_, _dt);
  const auto tau = 2.0 / wc;

  y_ = _dt < tau ? ((tau - _dt) * y_ + _dt * (_u + prev_u_)) / (tau + _dt) : _u;
  prev_u_ = _u;
}

template <typename T>
inline const T& LowPassFilter<T>::getValue() const
{
  return y_;
}

template <typename T>
inline void LowPassFilter<T>::setValue(const T& _x)
{
  y_ = prev_u_ = _x;
}

template <typename T>
void LowPassFilter<T>::setCutoffFrequency(double _fc_hz)
{
  if (_fc_hz <= 0.0) {
    wc_ = INFINITY;
  }
  else {
    wc_ = M_2PI * _fc_hz;  // Hz -> rad/s
  }
}
}  // namespace tobas::dsp
