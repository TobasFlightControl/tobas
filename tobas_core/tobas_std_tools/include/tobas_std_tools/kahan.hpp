#pragma once

namespace tobas_std
{
/* Kahan Summation. The worst-case round-off error scales with O(nε^2). */
template <typename T>
class Kahan
{
public:
  inline explicit Kahan();

  inline void add(const T& x);
  inline void reset();

  /* Request the summation. */
  inline const T& get() const;

private:
  T sum_ = 0;
  T c_ = 0;
};

template <typename T>
inline Kahan<T>::Kahan()
{
}

template <typename T>
inline void Kahan<T>::add(const T& x)
{
  const auto y = x - c_;
  const auto t = sum_ + y;
  c_ = (t - sum_) - y;
  sum_ = t;
}

template <typename T>
inline void Kahan<T>::reset()
{
  sum_ = 0;
  c_ = 0;
}

template <typename T>
inline const T& Kahan<T>::get() const
{
  return sum_;
}
}  // namespace tobas_std
