// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/tf_listener.hpp"

#include <tobas_kdl_conversions/kdl_msg.hpp>

namespace tobas
{
std::expected<kdl::Frame, std::string> TransformListener::lookupTransform(
  const std::string& _parent,
  const std::string& _child,
  const rclcpp::Time& _time) noexcept
{
  try {
    const auto transform = tf_buffer_.lookupTransform(_parent, _child, _time);
    kdl::transformMsgToKDL(transform.transform, frame_);
    return frame_;
  }
  catch (const tf2::TransformException& e) {
    return std::unexpected(std::string(e.what()));
  }
}
}  // namespace tobas
