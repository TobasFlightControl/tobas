#pragma once

#include <unordered_map>

#include <rclcpp/serialization.hpp>

namespace gui
{
namespace log
{
template <typename MsgType>
class MessageDecoder
{
public:
  explicit MessageDecoder();

  MsgType decode(rcutils_time_point_value_t time_ns, const rclcpp::SerializedMessage& ser_msg);

  void clearCache();

private:
  MsgType msg_;
  rclcpp::Serialization<MsgType> ser_;
  std::unordered_map<rcutils_time_point_value_t, MsgType> cache_map_;
};

template <typename MsgType>
MessageDecoder<MsgType>::MessageDecoder()
{
}

template <typename MsgType>
MsgType MessageDecoder<MsgType>::decode(rcutils_time_point_value_t time_ns, const rclcpp::SerializedMessage& ser_msg)
{
  if (cache_map_.contains(time_ns)) {
    return cache_map_[time_ns];
  }
  {
    ser_.deserialize_message(&ser_msg, &msg_);
    cache_map_[time_ns] = msg_;
    return msg_;
  }
}

template <typename MsgType>
void MessageDecoder<MsgType>::clearCache()
{
  cache_map_.clear();
}
}  // namespace log
}  // namespace gui
