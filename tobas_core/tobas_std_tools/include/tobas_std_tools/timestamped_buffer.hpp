#pragma once

#include <map>
#include <chrono>

namespace tobas_std
{
/**
 * @brief 指定した期間内のデータのみを保持するバッファ．
 * @tparam T データ型
 */
template <typename T>
class TimestampedBuffer
{
public:
  explicit TimestampedBuffer(const double& expiry_duration) : expiry_duration_(expiry_duration)
  {
  }

  void add(const std::chrono::system_clock::time_point& cur_time, const T& x)
  {
    buffer_[cur_time] = x;
    removeExpiredData(cur_time);
  }

  void clear()
  {
    buffer_.clear();
  }

  size_t size() const
  {
    return buffer_.size();
  }

protected:
  double expiry_duration_;
  std::map<std::chrono::system_clock::time_point, T> buffer_;

  void removeExpiredData(const std::chrono::system_clock::time_point& cur_time)
  {
    auto it = buffer_.begin();
    while (it != buffer_.end()
           && std::chrono::duration<double>(cur_time - it->first).count() > expiry_duration_)
      it = buffer_.erase(it);  // Erase returns the iterator following the removed element
  }
};

class TimestampedBufferDouble : public TimestampedBuffer<double>
{
public:
  using TimestampedBuffer<double>::TimestampedBuffer;

  double mean() const;
  double variance() const;
  double stddev() const;
};
}  // namespace tobas_std
