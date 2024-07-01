#pragma once

#include <limits>

namespace tobas_std
{
/* Stores the minimum value. */
template <typename T>
class MinimumHolder
{
public:
  inline explicit MinimumHolder()
  {
    reset();
  }

  inline void reset()
  {
    min_ = std::numeric_limits<T>::max();
  }

  inline void update(const T& x)
  {
    if (x < min_)
      min_ = x;
  }

  /* Request the minimum value. */
  inline const T& get() const
  {
    return min_;
  }

private:
  T min_;
};

/* Stores the maximum value. */
template <typename T>
class MaximumHolder
{
public:
  inline explicit MaximumHolder()
  {
    reset();
  }

  inline void reset()
  {
    max_ = std::numeric_limits<T>::lowest();
  }

  inline void update(const T& x)
  {
    if (x > max_)
      max_ = x;
  }

  /* Request the maximum value. */
  inline const T& get() const
  {
    return max_;
  }

private:
  T max_;
};

/* Stores the maximum and maximum values. */
template <typename T>
class RangeHolder
{
public:
  inline explicit RangeHolder()
  {
    reset();
  }

  inline void reset()
  {
    min_.reset();
    max_.reset();
  }

  inline void update(const T& x)
  {
    min_.update(x);
    max_.update(x);
  }

  inline const T& min() const
  {
    return min_.get();
  }

  inline const T& max() const
  {
    return max_.get();
  }

  inline T range() const
  {
    return max() - min();
  }

private:
  MinimumHolder<T> min_;
  MaximumHolder<T> max_;
};
}  // namespace tobas_std
