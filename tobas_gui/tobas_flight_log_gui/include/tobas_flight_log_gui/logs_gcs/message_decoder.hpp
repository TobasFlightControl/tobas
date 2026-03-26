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
class MessageDecoder
{
  using SerializedDataPtr = std::shared_ptr<rcutils_uint8_array_t>;

public:
  explicit MessageDecoder();

  const MsgType& decode(const SerializedDataPtr& ser_data);

private:
  MsgType msg_;
  rclcpp::Serialization<MsgType> ser_;
};

template <typename MsgType>
MessageDecoder<MsgType>::MessageDecoder()
{
}

template <typename MsgType>
const MsgType& MessageDecoder<MsgType>::decode(const SerializedDataPtr& ser_data)
{
  const rclcpp::SerializedMessage ser_msg(*ser_data);
  ser_.deserialize_message(&ser_msg, &msg_);
  return msg_;
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
