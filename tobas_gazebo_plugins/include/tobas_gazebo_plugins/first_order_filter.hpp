#pragma once

namespace gazebo
{
template <typename T>
class FirstOrderFilter
{
public:
  explicit FirstOrderFilter();

  void initialize(double time_const_up, double time_const_down, T init_state);

  /**
   * @brief This method will apply a first order filter on the input state.
   */
  T updateFilter(T input_state, double sampling_time);

private:
  bool is_initialized_;
  double time_const_up_;
  double time_const_down_;
  T prev_state_;
};

template <typename T>
FirstOrderFilter<T>::FirstOrderFilter() : is_initialized_(false)
{
}

template <typename T>
void FirstOrderFilter<T>::initialize(double time_const_up, double time_const_down, T init_state)
{
  time_const_up_ = time_const_up;
  time_const_down_ = time_const_down;
  prev_state_ = init_state;

  is_initialized_ = true;
}

template <typename T>
T FirstOrderFilter<T>::updateFilter(T input_state, double sampling_time)
{
  GZ_ASSERT(is_initialized_, "FirstOrderFilter is not initialized yet.");

  T output_state;
  if (input_state > prev_state_)
  {
    // Acceleration
    double alpha_up = exp(-sampling_time / time_const_up_);
    output_state = alpha_up * prev_state_ + (1 - alpha_up) * input_state;
  }
  else
  {
    // Deceleration
    double alpha_down = exp(-sampling_time / time_const_down_);
    output_state = alpha_down * prev_state_ + (1 - alpha_down) * input_state;
  }
  prev_state_ = output_state;
  return output_state;
}
}  // namespace gazebo
