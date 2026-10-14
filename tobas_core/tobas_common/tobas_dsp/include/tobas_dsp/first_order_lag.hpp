// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cassert>
#include <cmath>

#include "tobas_dsp/base_filter.hpp"

namespace tobas::dsp
{
/* First-order lag model: `tau * dx/dt = u - x`. */
template <typename T>
class FirstOrderLag : public BaseFilter<T>
{
public:
  explicit FirstOrderLag();

  void update(const T& _u, double _dt) override;

  inline const T& getValue() const override;
  inline void setValue(const T& _x) override;

  /** Set a finite, positive time constant in seconds without changing the state. */
  void setTimeConstant(double _tau);

private:
  double tau_ = INFINITY;  ///< [s]
  T x_{};
};

template <typename T>
FirstOrderLag<T>::FirstOrderLag()
{
}

template <typename T>
void FirstOrderLag<T>::update(const T& _u, double _dt)
{
  assert(std::isfinite(_dt) && _dt >= 0.0);
  if (_dt == 0.0) {
    return;
  }

  // `expm1` preserves accuracy when the update interval is small relative to the time constant.
  const auto gain = -std::expm1(-_dt / tau_);
  x_ += gain * (_u - x_);
}

template <typename T>
inline const T& FirstOrderLag<T>::getValue() const
{
  return x_;
}

template <typename T>
inline void FirstOrderLag<T>::setValue(const T& _x)
{
  x_ = _x;
}

template <typename T>
void FirstOrderLag<T>::setTimeConstant(double _tau)
{
  assert(_tau > 0.0);
  tau_ = _tau;
}
}  // namespace tobas::dsp
