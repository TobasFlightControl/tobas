// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>

namespace tobas::st
{
template <typename T>
class Range
{
public:
  T lower;
  T upper;

  inline Range(const T& _lower, const T& _upper);
  inline Range();

  inline void set(const T& _lower, const T& _upper);

  inline bool isValid() const;
  inline bool inRange(const T& _x, const T& _margin = 0) const;

  inline bool isStrictlyGreaterThan(const Range<T>& _other);
  inline bool isStrictlyLowerThan(const Range<T>& _other);
  inline bool isOverlapped(const Range<T>& _other);
  inline Range<T> overlappedArea(const Range<T>& _other);

  inline T clamp(const T& _x) const;

  /** Return true if value is clamped. */
  inline bool clamp(const T& _src, T& _dst) const;

  inline T range() const;
  inline T mean() const;

  inline std::pair<T, T> toPair() const;
  inline std::string toString() const;

  template <typename U>
  friend std::ostream& operator<<(std::ostream& _os, const Range<U>& _range);
};

template <typename T>
inline Range<T>::Range(const T& _lower, const T& _upper) : lower(_lower), upper(_upper)
{
  assert(_lower <= _upper);
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
inline bool Range<T>::inRange(const T& _x, const T& _margin) const
{
  return lower - _margin <= _x && _x <= upper + _margin;
}

template <typename T>
inline bool Range<T>::isOverlapped(const Range<T>& _other)
{
  return std::max(lower, _other.lower) <= std::min(upper, _other.upper);
}

template <typename T>
inline bool Range<T>::isStrictlyGreaterThan(const Range<T>& _other)
{
  return lower > _other.upper;
}

template <typename T>
inline bool Range<T>::isStrictlyLowerThan(const Range<T>& _other)
{
  return upper < _other.lower;
}

template <typename T>
inline Range<T> Range<T>::overlappedArea(const Range<T>& _other)
{
  return Range(std::max(lower, _other.lower), std::min(upper, _other.upper));
}

template <typename T>
inline T Range<T>::clamp(const T& _x) const
{
  return std::clamp(_x, lower, upper);
}

template <typename T>
inline bool Range<T>::clamp(const T& _src, T& _dst) const
{
  if (_src < lower) {
    _dst = lower;
    return true;
  }
  else if (_src > upper) {
    _dst = upper;
    return true;
  }
  else {
    _dst = _src;
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
inline std::pair<T, T> Range<T>::toPair() const
{
  return { lower, upper };
}

template <typename T>
inline std::string Range<T>::toString() const
{
  return "[" + std::to_string(lower) + ", " + std::to_string(upper) + "]";
}

template <typename U>
std::ostream& operator<<(std::ostream& _os, const Range<U>& _arg)
{
  _os << "[" << _arg.lower << ", " << _arg.upper << "]";
  return _os;
}
}  // namespace tobas::st
