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

  virtual void initialize(double time_const_up, double time_const_down, const T& init_value);

  virtual void update(const T& input_value, double sampling_time);

  virtual inline const T& getValue() const;

private:
  double time_const_up_ = INFINITY;
  double time_const_down_ = INFINITY;
  T value_ = {};
};

template <typename T>
AsymmetricFirstOrderFilter<T>::AsymmetricFirstOrderFilter()
{
}

template <typename T>
void AsymmetricFirstOrderFilter<T>::initialize(double time_const_up, double time_const_down, const T& init_value)
{
  assert(time_const_up > 0.0);
  assert(time_const_down > 0.0);

  time_const_up_ = time_const_up;
  time_const_down_ = time_const_down;
  value_ = init_value;
}

template <typename T>
void AsymmetricFirstOrderFilter<T>::update(const T& input_value, double sampling_time)
{
  assert(sampling_time >= 0.0);

  const auto time_const = input_value > value_ ? time_const_up_ : time_const_down_;
  const auto alpha = std::exp(-sampling_time / time_const);
  value_ = alpha * value_ + (1.0 - alpha) * input_value;
}

template <typename T>
inline const T& AsymmetricFirstOrderFilter<T>::getValue() const
{
  return value_;
}
}  // namespace gazebo
}  // namespace tobas
