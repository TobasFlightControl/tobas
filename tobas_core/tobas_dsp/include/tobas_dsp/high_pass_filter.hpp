#pragma once

#include <cmath>
#include <cassert>
#include <stdexcept>

#include "./utils.hpp"

namespace dsp
{
template <typename T>
class HighPassFilter
{
public:
  enum error_t : int
  {
    E_NO_ERROR = 0,
    E_TIME_STEP_NEGATIVE = -1,
    E_TIME_STEP_TOO_LARGE = -2,
  };

  explicit HighPassFilter();

  void initialize(const double& cutoff_freq, const T& init_value);
  error_t update(const T& u, const double& dt);

  inline const T& getOutput() const;

  inline error_t errorCode() const;
  inline const char* errorMessage() const;

private:
  error_t error_code_;

  double T_;
  T y_, prev_u_;
};

template <typename T>
HighPassFilter<T>::HighPassFilter()
{
}

template <typename T>
void HighPassFilter<T>::initialize(const double& cutoff_freq, const T& init_value)
{
  if (cutoff_freq <= 0)
    throw std::runtime_error("Cutoff frequency must be positive.");

  T_ = 2 * timeConstFromCutoff(cutoff_freq);
  y_ = prev_u_ = init_value;
}

template <typename T>
HighPassFilter<T>::error_t HighPassFilter<T>::update(const T& u, const double& dt)
{
  if (dt < 0.)
  {
    y_ = u;
    error_code_ = E_TIME_STEP_NEGATIVE;
  }
  else if (dt > T_)
  {
    y_ = u;
    error_code_ = E_TIME_STEP_TOO_LARGE;
  }
  else
  {
    y_ = ((T_ - dt) * y_ + T_ * (u - prev_u_)) / (T_ + dt);
    error_code_ = E_NO_ERROR;
  }

  prev_u_ = u;
  return error_code_;
}

template <typename T>
inline const T& HighPassFilter<T>::getOutput() const
{
  return y_;
}

template <typename T>
inline HighPassFilter<T>::error_t HighPassFilter<T>::errorCode() const
{
  return error_code_;
}

template <typename T>
inline const char* HighPassFilter<T>::errorMessage() const
{
  switch (error_code_)
  {
    case E_NO_ERROR:
      return "";
    case E_TIME_STEP_NEGATIVE:
      return "Time step must be positive.";
    case E_TIME_STEP_TOO_LARGE:
      return "Time step must be lower than the time constant.";
    default:
      return "Unknown error.";
  }
}
}  // namespace dsp
