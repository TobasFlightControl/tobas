#pragma once

#include <cmath>

namespace tobas
{
namespace dsp
{
template <typename T>
class IncompleteDifferentiation
{
private:
  /* data */
public:
  explicit IncompleteDifferentiation();
  explicit IncompleteDifferentiation(const double& cutoff_frequency);
  void setCutoffFrequency(const double& freq);
  void update(const T& u, const double& dt);
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
IncompleteDifferentiation<T>::IncompleteDifferentiation(const double& cutoff_frequency)
{
  setCutoffFrequency(cutoff_frequency);
}

template <typename T>
void IncompleteDifferentiation<T>::setCutoffFrequency(const double& freq)
{
  tau_ = 1 / (2.0 * M_PI * freq);
}

template <typename T>
void IncompleteDifferentiation<T>::update(const T& u, const double& dt)
{
  theta_ = (tau_ / dt * theta_ + (u - u_last_) / dt) / (1.0 + tau_ / dt);
  u_last_ = u;
}

template <typename T>
T IncompleteDifferentiation<T>::getValue()
{
  return theta_;
}

}  // namespace dsp
}  // namespace tobas
