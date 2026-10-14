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

  virtual void initialize(double _time_const, const T& _init_value);
};

template <typename T>
SymmetricFirstOrderFilter<T>::SymmetricFirstOrderFilter()
{
}

template <typename T>
void SymmetricFirstOrderFilter<T>::initialize(double _time_const, const T& _init_value)
{
  super::initialize(_time_const, _time_const, _init_value);
}
}  // namespace gazebo
}  // namespace tobas
