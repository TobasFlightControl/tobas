// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <future>

namespace tobas
{
namespace ros2
{
/* Similar to `future.wait_for()`, but waits indefinitely when the timeout is non-positive. */
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
