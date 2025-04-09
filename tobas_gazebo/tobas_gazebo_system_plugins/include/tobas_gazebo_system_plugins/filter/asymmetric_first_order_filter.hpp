#pragma once

#include <iostream>

namespace gazebo
{
/* 立ち上がりと立ち下がりで時定数が異なる一次遅れフィルタ． */
template <typename T>
class AsymmetricFirstOrderFilter
{
public:
  explicit AsymmetricFirstOrderFilter();

  bool initialize(const double& time_const_up, const double& time_const_down, const T& init_value);

  bool update(const T& input_value, const double& sampling_time);

  inline const T& getValue() const;

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
bool AsymmetricFirstOrderFilter<T>::initialize(
  const double& time_const_up,
  const double& time_const_down,
  const T& init_value)
{
  if (time_const_up < 0. || time_const_down < 0.)
  {
    std::cerr << "Time constant must be non-negative." << std::endl;
    return false;
  }

  time_const_up_ = time_const_up;
  time_const_down_ = time_const_down;
  value_ = init_value;

  is_initialized_ = true;
  return true;
}

template <typename T>
bool AsymmetricFirstOrderFilter<T>::update(const T& input_value, const double& sampling_time)
{
  if (!is_initialized_)
  {
    std::cerr << "Filter is not initialized yet." << std::endl;
    return false;
  }

  if (sampling_time <= 0.)
  {
    std::cerr << "Sampling time must be positive." << std::endl;
    return false;
  }

  if (input_value > value_)
  {
    // Acceleration
    const double alpha_up = time_const_up_ > 0 ? exp(-sampling_time / time_const_up_) : 0;
    value_ = alpha_up * value_ + (1 - alpha_up) * input_value;
  }
  else
  {
    // Deceleration
    const double alpha_down = time_const_down_ > 0 ? exp(-sampling_time / time_const_down_) : 0;
    value_ = alpha_down * value_ + (1 - alpha_down) * input_value;
  }

  return true;
}

template <typename T>
inline const T& AsymmetricFirstOrderFilter<T>::getValue() const
{
  return value_;
}
}  // namespace gazebo
