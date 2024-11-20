#pragma once

namespace dsp
{
template <typename T>
class BaseFilter
{
public:
  enum error_t : int
  {
    E_NO_ERROR = 0,
    E_TIME_STEP_NEGATIVE = -1,
    E_TIME_STEP_TOO_LARGE = -2,
  };

  virtual error_t update(const T& u, const double& dt) = 0;

  virtual const T& getValue() const = 0;
  virtual void setValue(const T& x) = 0;

  virtual inline error_t errorCode() const;
  virtual inline const char* errorMessage() const;

protected:
  error_t error_code_ = E_NO_ERROR;
};

template <typename T>
inline BaseFilter<T>::error_t BaseFilter<T>::errorCode() const
{
  return error_code_;
}

template <typename T>
inline const char* BaseFilter<T>::errorMessage() const
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
