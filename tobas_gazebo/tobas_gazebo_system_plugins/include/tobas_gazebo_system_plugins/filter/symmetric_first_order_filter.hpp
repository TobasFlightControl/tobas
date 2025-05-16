#pragma once

#include "./asymmetric_first_order_filter.hpp"

namespace gazebo
{
template <typename T>
class SymmetricFirstOrderFilter : public AsymmetricFirstOrderFilter<T>
{
  using super = AsymmetricFirstOrderFilter<T>;

public:
  explicit SymmetricFirstOrderFilter();

  bool initialize(const double& time_const, const T& init_value);
};

template <typename T>
SymmetricFirstOrderFilter<T>::SymmetricFirstOrderFilter()
{
}

template <typename T>
bool SymmetricFirstOrderFilter<T>::initialize(const double& time_const, const T& init_value)
{
  return super::initialize(time_const, time_const, init_value);
}
}  // namespace gazebo
