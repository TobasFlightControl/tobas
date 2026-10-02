// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./asymmetric_first_order_filter.hpp"

namespace tobas
{
namespace gazebo
{
template <typename T>
class SymmetricFirstOrderFilter : public AsymmetricFirstOrderFilter<T>
{
  using super = AsymmetricFirstOrderFilter<T>;

public:
  explicit SymmetricFirstOrderFilter();

  using super::initialize;
  virtual void initialize(double time_const, const T& init_value);
};

template <typename T>
SymmetricFirstOrderFilter<T>::SymmetricFirstOrderFilter()
{
}

template <typename T>
void SymmetricFirstOrderFilter<T>::initialize(double time_const, const T& init_value)
{
  super::initialize(time_const, time_const, init_value);
}
}  // namespace gazebo
}  // namespace tobas
