#pragma once

#include <cmath>

namespace tobas::dsp
{
template <typename T>
class IncompleteDifferentiation
{
private:
  /* data */
public:
  explicit IncompleteDifferentiation();
  explicit IncompleteDifferentiation(double _cutoff_frequency);
  void setCutoffFrequency(double _freq);
  void update(const T& _u, double _dt);
  T getValue();

private:
  double tau_ = 0.0;
  T theta_{};
  T u_last_{};
};

template <typename T>
IncompleteDifferentiation<T>::IncompleteDifferentiation()
{
}

template <typename T>
IncompleteDifferentiation<T>::IncompleteDifferentiation(double _cutoff_frequency)
{
  setCutoffFrequency(_cutoff_frequency);
}

template <typename T>
void IncompleteDifferentiation<T>::setCutoffFrequency(double _freq)
{
  tau_ = 1 / (2.0 * M_PI * _freq);
}

template <typename T>
void IncompleteDifferentiation<T>::update(const T& _u, double _dt)
{
  theta_ = (tau_ / _dt * theta_ + (_u - u_last_) / _dt) / (1.0 + tau_ / _dt);
  u_last_ = _u;
}

template <typename T>
T IncompleteDifferentiation<T>::getValue()
{
  return theta_;
}
}  // namespace tobas::dsp
