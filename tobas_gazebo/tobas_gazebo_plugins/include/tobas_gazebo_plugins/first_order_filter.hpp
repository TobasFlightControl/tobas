#pragma once

#include <cassert>

namespace gazebo
{
/* 立ち上がりと立ち下がりで時定数が異なる一次遅れフィルタ． */
template <typename T>
class AsymmetricFirstOrderFilter
{
public:
  explicit AsymmetricFirstOrderFilter();

  void initialize(const double& time_const_up, const double& time_const_down, T init_state);

  T update(T input_state, const double& sampling_time);

private:
  bool is_initialized_ = false;
  double time_const_up_;
  double time_const_down_;
  T prev_state_;
};

template <typename T>
AsymmetricFirstOrderFilter<T>::AsymmetricFirstOrderFilter()
{
}

template <typename T>
void AsymmetricFirstOrderFilter<T>::initialize(const double& time_const_up, const double& time_const_down, T init_state)
{
  assert(time_const_up >= 0);
  assert(time_const_down >= 0);

  time_const_up_ = time_const_up;
  time_const_down_ = time_const_down;
  prev_state_ = init_state;

  is_initialized_ = true;
}

template <typename T>
T AsymmetricFirstOrderFilter<T>::update(T input_state, const double& sampling_time)
{
  assert(is_initialized_);
  assert(sampling_time >= 0);

  T output_state;
  if (input_state > prev_state_)
  {
    // Acceleration
    const double alpha_up = time_const_up_ > 0 ? exp(-sampling_time / time_const_up_) : 0;
    output_state = alpha_up * prev_state_ + (1 - alpha_up) * input_state;
  }
  else
  {
    // Deceleration
    const double alpha_down = time_const_down_ > 0 ? exp(-sampling_time / time_const_down_) : 0;
    output_state = alpha_down * prev_state_ + (1 - alpha_down) * input_state;
  }
  prev_state_ = output_state;
  return output_state;
}
}  // namespace gazebo
