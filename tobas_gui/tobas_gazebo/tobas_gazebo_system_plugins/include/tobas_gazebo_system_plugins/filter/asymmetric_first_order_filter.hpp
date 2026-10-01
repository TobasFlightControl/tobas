// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cassert>
#include <cmath>

namespace tobas
{
namespace gazebo
{
/** First-order lag filter with different time constants for rise and fall. */
template <typename T>
class AsymmetricFirstOrderFilter
{
public:
  explicit AsymmetricFirstOrderFilter();

  virtual void initialize(const double& time_const_up, const double& time_const_down, const T& init_value);

  virtual void update(const T& input_value, const double& sampling_time);

  virtual inline const T& getValue() const;

private:
  bool is_initialized_ = false;
  double time_const_up_;
  double time_const_down_;
  T value_;
};

template <typename T>
AsymmetricFirstOrderFilter<T>::AsymmetricFirstOrderFilter()
{
}

template <typename T>
void AsymmetricFirstOrderFilter<T>::initialize(
  const double& time_const_up,
  const double& time_const_down,
  const T& init_value)
{
  assert(time_const_up >= 0.0 && "Rise time constant must be non-negative.");
  assert(time_const_down >= 0.0 && "Fall time constant must be non-negative.");

  time_const_up_ = time_const_up;
  time_const_down_ = time_const_down;
  value_ = init_value;

  is_initialized_ = true;
}

template <typename T>
void AsymmetricFirstOrderFilter<T>::update(const T& input_value, const double& sampling_time)
{
  assert(is_initialized_ && "Filter must be initialized before updating.");
  assert(sampling_time > 0.0 && "Sampling time must be positive.");

  if (input_value > value_) {
    // Acceleration
    const double alpha_up = time_const_up_ > 0 ? std::exp(-sampling_time / time_const_up_) : 0;
    value_ = alpha_up * value_ + (1 - alpha_up) * input_value;
  }
  else {
    // Deceleration
    const double alpha_down = time_const_down_ > 0 ? std::exp(-sampling_time / time_const_down_) : 0;
    value_ = alpha_down * value_ + (1 - alpha_down) * input_value;
  }
}

template <typename T>
inline const T& AsymmetricFirstOrderFilter<T>::getValue() const
{
  return value_;
}
}  // namespace gazebo
}  // namespace tobas
