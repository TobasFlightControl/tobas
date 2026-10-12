// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <vector>

namespace tobas
{
namespace st
{
template <typename T>
class Buffer
{
public:
  explicit Buffer(size_t _max_size = 1);

  void resize(size_t _max_size);
  void add(T _item);
  void fill(T _item);

  inline size_t size() const;
  inline size_t maxSize() const;
  inline bool isFull() const;
  inline bool isEmpty() const;
  inline const T& get(size_t _idx) const;
  inline const T& getLatest() const;
  inline const T& getOldest() const;

private:
  size_t max_size_;
  size_t cur_;
  std::vector<T> data_;
};

template <typename T>
Buffer<T>::Buffer(size_t _max_size)
{
  resize(_max_size);
}

template <typename T>
void Buffer<T>::resize(size_t _max_size)
{
  assert(_max_size > 0);

  max_size_ = _max_size;
  cur_ = 0;
  data_.resize(_max_size);
}

template <typename T>
void Buffer<T>::add(T _item)
{
  data_[cur_ % max_size_] = _item;
  ++cur_;
}

template <typename T>
void Buffer<T>::fill(T _item)
{
  for (int i = 0; i < max_size_; ++i) {
    data_[i] = _item;
  }
  cur_ = max_size_;
}

template <typename T>
inline size_t Buffer<T>::size() const
{
  return isFull() ? max_size_ : cur_;
}

template <typename T>
inline size_t Buffer<T>::maxSize() const
{
  return max_size_;
}

template <typename T>
inline bool Buffer<T>::isFull() const
{
  return cur_ >= max_size_;
}

template <typename T>
inline bool Buffer<T>::isEmpty() const
{
  return size() == 0;
}

template <typename T>
inline const T& Buffer<T>::get(size_t _idx) const
{
  assert(_idx < std::min(max_size_, cur_));
  return data_[_idx];
}

template <typename T>
inline const T& Buffer<T>::getLatest() const
{
  assert(cur_ > 0);
  return get((cur_ - 1) % max_size_);
}

template <typename T>
inline const T& Buffer<T>::getOldest() const
{
  assert(cur_ > 0);
  return isFull() ? get(cur_ % max_size_) : get(0);
}
}  // namespace st
}  // namespace tobas
