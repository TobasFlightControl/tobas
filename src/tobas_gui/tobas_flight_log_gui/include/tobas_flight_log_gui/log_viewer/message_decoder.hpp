// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <exception>
#include <expected>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/serialization.hpp>

namespace tobas::gui::log
{
template <typename MsgType>
class MessageDecoderCache
{
  using Time = rcutils_time_point_value_t;
  using SerializedDataPtr = std::shared_ptr<rcutils_uint8_array_t>;

public:
  explicit MessageDecoderCache();

  std::expected<const MsgType*, std::string> decode(Time _time_ns, const SerializedDataPtr& _ser_data);

  void clearCache();

  void retainWindow(Time _start, Time _stop);

private:
  rclcpp::Serialization<MsgType> ser_;
  std::map<Time, MsgType> cache_map_;
};

template <typename MsgType>
MessageDecoderCache<MsgType>::MessageDecoderCache()
{
}

template <typename MsgType>
std::expected<const MsgType*, std::string>
MessageDecoderCache<MsgType>::decode(Time _time_ns, const SerializedDataPtr& _ser_data)
{
  const auto [it, inserted] = cache_map_.try_emplace(_time_ns);
  if (inserted) {
    try {
      const rclcpp::SerializedMessage ser_msg(*_ser_data);
      ser_.deserialize_message(&ser_msg, &it->second);
    }
    catch (const std::exception& e) {
      cache_map_.erase(it);
      return std::unexpected(e.what());
    }
    catch (...) {
      cache_map_.erase(it);
      return std::unexpected("Unknown error while deserializing message.");
    }
  }
  return &it->second;
}

template <typename MsgType>
void MessageDecoderCache<MsgType>::clearCache()
{
  cache_map_.clear();
}

template <typename MsgType>
void MessageDecoderCache<MsgType>::retainWindow(Time _start, Time _stop)
{
  cache_map_.erase(cache_map_.begin(), cache_map_.lower_bound(_start));
  cache_map_.erase(cache_map_.upper_bound(_stop), cache_map_.end());
}
}  // namespace tobas::gui::log
