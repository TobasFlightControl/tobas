#pragma once

#include <unordered_map>

#include <rclcpp/serialization.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
template <typename MsgType>
class MessageDecoderCache
{
  using Time = rcutils_time_point_value_t;
  using SerializedDataPtr = std::shared_ptr<rcutils_uint8_array_t>;

public:
  explicit MessageDecoderCache();

  const MsgType& decode(const Time& time_ns, const SerializedDataPtr& ser_data);

  void clearCache();

private:
  MsgType msg_;
  rclcpp::Serialization<MsgType> ser_;
  std::unordered_map<Time, MsgType> cache_map_;
};

template <typename MsgType>
MessageDecoderCache<MsgType>::MessageDecoderCache()
{
}

template <typename MsgType>
const MsgType& MessageDecoderCache<MsgType>::decode(const Time& time_ns, const SerializedDataPtr& ser_data)
{
  if (cache_map_.contains(time_ns)) {
    return cache_map_[time_ns];
  }

  const rclcpp::SerializedMessage ser_msg(*ser_data);
  ser_.deserialize_message(&ser_msg, &msg_);
  return cache_map_[time_ns] = msg_;
}

template <typename MsgType>
void MessageDecoderCache<MsgType>::clearCache()
{
  cache_map_.clear();
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
