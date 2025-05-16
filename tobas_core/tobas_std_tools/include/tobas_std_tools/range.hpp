#pragma once

#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>

namespace tobas_std
{
template <typename T>
class Range
{
public:
  T lower;
  T upper;

  inline Range(const T& lower, const T& upper);
  inline Range();

  inline void set(const T& lower, const T& upper);

  inline bool isValid() const;
  inline bool inRange(const T& x, const T& margin = 0) const;

  inline bool isStrictlyGreaterThan(const Range<T>& other);
  inline bool isStrictlyLowerThan(const Range<T>& other);
  inline bool isOverlapped(const Range<T>& other);
  inline Range<T> overlappedArea(const Range<T>& other);

  inline T clamp(const T& x) const;

  /* Return true if value is clamped. */
  inline bool clamp(const T& src, T& des) const;

  inline T range() const;
  inline T mean() const;

  inline std::string toString() const;

  template <typename U>
  friend std::ostream& operator<<(std::ostream& os, const Range<U>& range);
};

template <typename T>
inline Range<T>::Range(const T& lower, const T& upper) : lower(lower), upper(upper)
{
  assert(lower <= upper);
}

template <typename T>
inline Range<T>::Range() : lower(0), upper(0)
{
}

template <typename T>
inline void Range<T>::set(const T& _lower, const T& _upper)
{
  lower = _lower;
  upper = _upper;
}

template <typename T>
inline bool Range<T>::isValid() const
{
  return lower <= upper;
}

template <typename T>
inline bool Range<T>::inRange(const T& x, const T& margin) const
{
  return lower - margin <= x && x <= upper + margin;
}

template <typename T>
inline bool Range<T>::isOverlapped(const Range<T>& other)
{
  return std::max(lower, other.lower) <= std::min(upper, other.upper);
}

template <typename T>
inline bool Range<T>::isStrictlyGreaterThan(const Range<T>& other)
{
  return lower > other.upper;
}

template <typename T>
inline bool Range<T>::isStrictlyLowerThan(const Range<T>& other)
{
  return upper < other.lower;
}

template <typename T>
inline Range<T> Range<T>::overlappedArea(const Range<T>& other)
{
  return Range(std::max(lower, other.lower), std::min(upper, other.upper));
}

template <typename T>
inline T Range<T>::clamp(const T& x) const
{
  return std::clamp(x, lower, upper);
}

template <typename T>
inline bool Range<T>::clamp(const T& src, T& des) const
{
  if (src < lower) {
    des = lower;
    return true;
  }
  else if (src > upper) {
    des = upper;
    return true;
  }
  else {
    des = src;
    return false;
  }
}

template <typename T>
inline T Range<T>::range() const
{
  return upper - lower;
}

template <typename T>
inline T Range<T>::mean() const
{
  return (lower + upper) / 2;
}

template <typename T>
inline std::string Range<T>::toString() const
{
  return "[" + std::to_string(lower) + ", " + std::to_string(upper) + "]";
}

template <typename U>
std::ostream& operator<<(std::ostream& os, const Range<U>& arg)
{
  os << "[" << arg.lower << ", " << arg.upper << "]";
  return os;
}
}  // namespace tobas_std
