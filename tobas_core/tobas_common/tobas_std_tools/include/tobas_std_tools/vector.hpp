// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <algorithm>
#include <cassert>
#include <iostream>
#include <unordered_set>
#include <vector>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_math/core.hpp>

namespace std
{
/** Console output for `std::vector`. */
template <typename T>
std::ostream& operator<<(std::ostream& _os, const std::vector<T>& _vec)
{
  const auto size = _vec.size();

  if (size == 0) {
    _os << "[]";
    return _os;
  }

  _os << "[";
  for (size_t i = 0; i < size - 1; ++i) {
    _os << _vec[i] << " ";
  }
  _os << _vec[size - 1] << "]";

  return _os;
}
}  // namespace std

namespace tobas
{
namespace st
{
/** Naive Summation． The worst-case round-off error scales with O(nε). */
template <typename T>
T sum(const std::vector<T>& _arr)
{
  T sum = 0;
  for (const auto& x : _arr) {
    sum += x;
  }
  return sum;
}

/** Naive Summation． The worst-case round-off error scales with O(nε). */
template <typename T>
T sum(const std::vector<T>& _arr, size_t _start, size_t _size)
{
  const auto stop = _start + _size;
  assert(stop <= _arr.size());

  T sum = 0;
  for (size_t i = _start; i < stop; ++i) {
    sum += _arr[i];
  }
  return sum;
}

/** Kahan Summation. The worst-case round-off error scales with O(nε^2). */
template <typename T>
T fsum(const std::vector<T>& _arr)
{
  algo::Kahan<T> sum;
  for (const auto& x : _arr) {
    sum.add(x);
  }
  return sum.get();
}

/** Kahan Summation. The worst-case round-off error scales with O(nε^2). */
template <typename T>
T fsum(const std::vector<T>& _arr, size_t _start, size_t _size)
{
  const auto stop = _start + _size;
  assert(stop <= _arr.size());

  algo::Kahan<T> sum;
  for (size_t i = _start; i < stop; ++i) {
    sum.add(_arr[i]);
  }
  return sum.get();
}

/** The average of Kahan Summation. */
template <typename T>
T fmean(const std::vector<T>& _arr)
{
  if (_arr.empty()) {
    return 0;
  }

  return fsum(_arr) / _arr.size();
}

/** The average of Kahan Summation. */
template <typename T>
T fmean(const std::vector<T>& _arr, size_t _start, size_t _size)
{
  if (_size == 0) {
    return 0;
  }

  return fsum(_arr, _start, _size) / _size;
}

/** The variance of data. */
template <typename T>
T variance(const std::vector<T>& _arr)
{
  if (_arr.empty()) {
    return 0;
  }

  const auto mean = fmean(_arr);
  algo::Kahan<T> sum;
  for (const auto& x : _arr) {
    sum.add(math::sqr(x - mean));
  }
  return sum.get() / _arr.size();
}

/** The variance of data. */
template <typename T>
T variance(const std::vector<T>& _arr, size_t _start, size_t _size)
{
  const auto stop = _start + _size;
  assert(stop <= _arr.size());

  if (_arr.empty()) {
    return 0;
  }

  const auto mean = fmean(_arr, _start, _size);
  algo::Kahan<T> sum;
  for (size_t i = _start; i < stop; ++i) {
    sum.add(math::sqr(_arr[i] - mean));
  }
  return sum.get() / _size;
}

/** Calculate the weighted average of elements. */
template <typename T, typename U>
T average(const std::vector<T>& _vec, const std::vector<U>& _weights)
{
  assert(_vec.size() == _weights.size());

  T num = 0;
  U den = 0;
  for (size_t i = 0; i < _vec.size(); ++i) {
    num += _vec[i] * _weights[i];
    den += _weights[i];
  }

  T res = num / den;
  return res;
}

/** Calculate the average of elements. */
template <typename T>
inline T average(const std::vector<T>& _vec)
{
  return average(_vec, std::vector<double>(_vec.size(), 1.0));
}

/** Get the element index from a `std::vector`. */
template <typename T>
ssize_t findIndex(const std::vector<T>& _vec, const T& _item)
{
  const auto ret = std::find(_vec.begin(), _vec.end(), _item);
  if (ret == _vec.end()) {
    return -1;
  }
  return ret - _vec.begin();
}

/** Fill all elements with a single value. */
template <typename T>
inline void fill(std::vector<T>& _vec, const T& _item)
{
  std::fill(_vec.begin(), _vec.end(), _item);
}

/** Return true when all elements are greater than `a`. */
template <typename T>
bool all_gt(const std::vector<T>& _vec, const T& _a)
{
  for (const auto& x : _vec) {
    if (x <= _a) {
      return false;
    }
  }
  return true;
}

/** Return true when all elements are less than `a`. */
template <typename T>
bool all_lt(const std::vector<T>& _vec, const T& _a)
{
  for (const auto& x : _vec) {
    if (x >= _a) {
      return false;
    }
  }
  return true;
}

/** Return true when all elements are greater than or equal to `a`. */
template <typename T>
bool all_ge(const std::vector<T>& _vec, const T& _a)
{
  for (const auto& x : _vec) {
    if (x < _a) {
      return false;
    }
  }
  return true;
}

/** Return true when all elements are less than or equal to `a`. */
template <typename T>
bool all_le(const std::vector<T>& _vec, const T& _a)
{
  for (const auto& x : _vec) {
    if (x > _a) {
      return false;
    }
  }
  return true;
}

/** Return the index of the closest value. */
template <typename T>
size_t closestIndex(const std::vector<T>& _vec, const T& _a)
{
  assert(!_vec.empty());

  size_t closest_idx = 0;  // Initialized to avoid compiler warnings.
  T closest_dist = std::numeric_limits<T>::max();

  for (size_t i = 0; i < _vec.size(); ++i) {
    const T dist = std::abs(_vec[i] - _a);
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}

/** Remove duplicate elements. */
template <typename T>
std::vector<T> unique(const std::vector<T>& _vec)
{
  std::unordered_set<T> seen;
  std::vector<T> res;

  for (const auto& val : _vec) {
    if (seen.find(val) == seen.end()) {
      seen.insert(val);
      res.push_back(val);
    }
  }

  return res;
}

/** Return true when all elements are unique. */
template <typename T>
inline bool isUnique(const std::vector<T>& _vec)
{
  return unique(_vec).size() == _vec.size();
}

/** Return true when the element is included. */
template <typename T>
inline bool contains(const std::vector<T>& _vec, const T& _val)
{
  return std::find(_vec.begin(), _vec.end(), _val) != _vec.end();
}

/** Return true when all elements satisfy the condition. */
template <typename T, typename Lambda>
inline bool allOf(const std::vector<T>& _vec, const Lambda& _lambda)
{
  return all_of(_vec.begin(), _vec.end(), _lambda);
}

/** Merge two `std::vector`s. */
template <typename T>
std::vector<T> merge(const std::vector<T>& _vec1, const std::vector<T>& _vec2)
{
  std::vector<T> res = _vec1;
  res.insert(res.end(), _vec2.begin(), _vec2.end());
  return res;
}

/** Return the element index. */
template <typename T>
ssize_t index(const std::vector<T>& _vec, const T& _value)
{
  const auto it = std::find(_vec.begin(), _vec.end(), _value);
  if (it != _vec.end()) {
    return std::distance(_vec.begin(), it);
  }
  else {
    return -1;
  }
}

/** Remove the element at the specified index (O(N)). */
template <typename T>
void eraseIndex(std::vector<T>& _vec, size_t _idx)
{
  assert(_idx < _vec.size());
  _vec.erase(_vec.begin() + static_cast<std::ptrdiff_t>(_idx));
}
}  // namespace st
}  // namespace tobas
