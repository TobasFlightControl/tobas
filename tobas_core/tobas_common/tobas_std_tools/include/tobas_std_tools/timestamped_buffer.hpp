// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cassert>
#include <chrono>
#include <map>

namespace tobas
{
namespace st
{
/* Buffer that keeps only data within the specified duration. */
template <typename T>
class TimestampedBuffer
{
  using TimeType = std::chrono::steady_clock::time_point;
  using DurationType = std::chrono::milliseconds;
  using MapType = std::map<TimeType, T>;

public:
  explicit TimestampedBuffer(const DurationType& expiry_duration) : expiry_duration_(expiry_duration)
  {
    assert(expiry_duration >= DurationType(0));
  }

  void add(const TimeType& cur_time, const T& x)
  {
    map_[cur_time] = x;
    removeExpiredData(cur_time);
  }

  void clear()
  {
    map_.clear();
    is_filled_ = false;
  }

  size_t size() const
  {
    return map_.size();
  }

  const bool& isFilled() const
  {
    return is_filled_;
  }

  typename MapType::const_iterator first() const
  {
    assert(!map_.empty());
    return map_.begin();
  }

  typename MapType::const_iterator last() const
  {
    assert(!map_.empty());
    return std::prev(map_.end());
  }

  const TimeType& firstTime() const
  {
    return first()->first;
  }

  const T& firstValue() const
  {
    return first()->second;
  }

  const TimeType& lastTime() const
  {
    return last()->first;
  }

  const T& lastValue() const
  {
    return last()->second;
  }

  /* Get the first value at or after the given time. */
  const T& closestAfterValue(const TimeType& time) const
  {
    assert(!map_.empty());

    // Get the first element at or after the given time.
    // `lower_bound` and `upper_bound` do not bracket a key.
    // The former returns an element with a key greater than or equal to the key,
    // while the latter returns an element with a key greater than the key.
    auto it = map_.lower_bound(time);

    // Return the latest value if the element is too old.
    if (it == map_.end()) {
      --it;
    }

    return it->second;
  }

protected:
  const DurationType expiry_duration_;
  MapType map_;
  bool is_filled_ = false;

  void removeExpiredData(const TimeType& cur_time)
  {
    auto it = map_.begin();
    while (it != map_.end() && cur_time - it->first > expiry_duration_) {
      it = map_.erase(it);  // Erase returns the iterator following the removed element
      is_filled_ = true;
    }
  }
};

class TimestampedBufferDouble : public TimestampedBuffer<double>
{
public:
  using TimestampedBuffer<double>::TimestampedBuffer;

  double max() const;
  double min() const;
  double range() const;
  double mean() const;
  double variance() const;
  double stddev() const;
};
}  // namespace st
}  // namespace tobas
