// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cassert>
#include <cmath>
#include <iostream>

#include <tobas_math/definitions.hpp>

#include "./base_filter.hpp"
#include "./utils.hpp"

namespace tobas
{
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

  void setCutoffFrequency(const double& fc_hz);

private:
  double wc_ = INFINITY;  // [rad/s]
  T y_{};
  T prev_u_{};
};

template <typename T>
HighPassFilter<T>::HighPassFilter()
{
}

template <typename T>
void HighPassFilter<T>::update(const T& u, const double& dt)
{
  assert(dt >= 0.0);

  const auto wc = prewarp(wc_, dt);
  const auto tau = 2.0 / wc;

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
void HighPassFilter<T>::setCutoffFrequency(const double& fc_hz)
{
  if (fc_hz <= 0.0) {
    wc_ = INFINITY;
  }
  else {
    wc_ = M_2PI * fc_hz;  // Hz -> rad/s
  }
}
}  // namespace dsp
}  // namespace tobas
