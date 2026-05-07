// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/client.hpp>

namespace tobas
{
namespace ros2
{
/* future.wait_for()とほぼ同じだが，タイムアウトが非正の場合は無限待機する． */
template <typename FutureType, typename RepType, typename DurType>
std::future_status waitForFuture(const FutureType& future, std::chrono::duration<RepType, DurType> timeout)
{
  if (timeout.count() > 0) {
    return future.wait_for(timeout);
  }
  else {
    future.wait();
    return std::future_status::ready;
  }
}
}  // namespace ros2
}  // namespace tobas
