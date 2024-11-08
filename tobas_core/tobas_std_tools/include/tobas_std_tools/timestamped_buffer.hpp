#pragma once

#include <map>
#include <chrono>
#include <cassert>

namespace tobas_std
{
/**
 * @brief 指定した期間内のデータのみを保持するバッファ．
 * @tparam T データ型
 */
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
    assert(map_.size() > 0);
    return map_.begin();
  }

  typename MapType::const_iterator last() const
  {
    assert(map_.size() > 0);
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

  /* 与えられた時刻以後の最初の値を取得する． */
  const T& closestAfterValue(const TimeType& time) const
  {
    assert(map_.size() > 0);

    // 与えられた時刻以後の最初の要素を取得
    // XXX: lower_bound, upper_boundはキーを挟んでいるわけではなく，前者はキー以上，後者はキーより大きい要素を返す．
    auto it = map_.lower_bound(time);

    // 要素が古すぎる場合は最新の値を返す
    if (it == map_.end())
      --it;

    return it->second;
  }

protected:
  const DurationType expiry_duration_;
  MapType map_;
  bool is_filled_ = false;

  void removeExpiredData(const TimeType& cur_time)
  {
    auto it = map_.begin();
    while (it != map_.end() && cur_time - it->first > expiry_duration_)
    {
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
}  // namespace tobas_std
