#pragma once

#include <vector>
#include <string>
#include <cassert>

#include "./assert.hpp"

namespace dh_std
{
template <typename T>
class Buffer
{
public:
  explicit Buffer(size_t max_size = 1);

  void resize(size_t max_size);
  void add(T item);
  void fill(T item);

  inline size_t size() const;
  inline size_t maxSize() const;
  inline bool isFull() const;
  inline bool isEmpty() const;
  inline const T& get(size_t idx) const;
  inline const T& getLatest() const;
  inline const T& getOldest() const;

private:
  size_t max_size_;
  size_t cur_;
  std::vector<T> data_;
};

template <typename T>
Buffer<T>::Buffer(size_t max_size)
{
  resize(max_size);
}

template <typename T>
void Buffer<T>::resize(size_t max_size)
{
  assert(max_size > 0);

  max_size_ = max_size;
  cur_ = 0;
  data_.resize(max_size);
}

template <typename T>
void Buffer<T>::add(T item)
{
  data_[cur_ % max_size_] = item;
  ++cur_;
}

template <typename T>
void Buffer<T>::fill(T item)
{
  for (int i = 0; i < max_size_; ++i)
  {
    data_[i] = item;
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
inline const T& Buffer<T>::get(size_t idx) const
{
  assertWithMsg(
    idx < std::min(max_size_, cur_),
    "Invalid index: idx = " << idx << ", max_size = " << max_size_ << ", current index = " << cur_);
  return data_[idx];
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
}  // namespace dh_std
