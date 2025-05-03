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

  const MsgType& decode(long time, const rclcpp::SerializedMessage& ser_msg);

  void clearCache();

private:
  MsgType msg_;
  rclcpp::Serialization<MsgType> ser_;
  std::unordered_map<long, MsgType> cache_map_;
};

template <typename MsgType>
MessageDecoder<MsgType>::MessageDecoder()
{
}

template <typename MsgType>
const MsgType& MessageDecoder<MsgType>::decode(long time, const rclcpp::SerializedMessage& ser_msg)
{
  if (cache_map_.contains(time)) {
    return cache_map_[time];
  }
  {
    ser_.deserialize_message(&ser_msg, &msg_);
    cache_map_[time] = msg_;
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
