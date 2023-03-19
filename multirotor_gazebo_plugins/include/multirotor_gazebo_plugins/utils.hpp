#pragma once

#include <gazebo/gazebo.hh>

/**
 * @brief utilities
 * @note 各プラグインは個別にビルドしてリンクは行っていないため，ヘッダオンリーで書く．
 */
namespace gazebo
{
template <typename T>
inline T sqr(const T& x)
{
  return x * x;
}

template <typename T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value)
{
  if (sdf->HasElement(name))
  {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  }
  else
  {
    param = default_value;
    return false;
  }
}

template <typename T>
bool allGreaterEqual(const ignition::math::Vector3<T>& v, T x)
{
  return v.X() >= x && v.Y() >= x && v.Z() >= x;
}

template <typename T>
class FirstOrderFilter
{
public:
  FirstOrderFilter() : is_initialized_(false)
  {
  }

  void initialize(double time_const_up, double time_const_down, T init_state)
  {
    time_const_up_ = time_const_up;
    time_const_down_ = time_const_down;
    prev_state_ = init_state;

    is_initialized_ = true;
  }

  /**
   * @brief This method will apply a first order filter on the input_state.
   */
  T updateFilter(T input_state, double sampling_time)
  {
    GZ_ASSERT(is_initialized_, "FirstOrderFilter is not initialized yet.");

    T output_state_;
    if (input_state > prev_state_)
    {
      // Calcuate the output_state_ if accelerating.
      double alpha_up = exp(-sampling_time / time_const_up_);
      // x(k+1) = Ad*x(k) + Bd*u(k)
      output_state_ = alpha_up * prev_state_ + (1 - alpha_up) * input_state;
    }
    else
    {
      // Calculate the output_state_ if decelerating.
      double alphaDown = exp(-sampling_time / time_const_down_);
      output_state_ = alphaDown * prev_state_ + (1 - alphaDown) * input_state;
    }
    prev_state_ = output_state_;
    return output_state_;
  }

private:
  bool is_initialized_;
  double time_const_up_;
  double time_const_down_;
  T prev_state_;
};
}  // namespace gazebo
