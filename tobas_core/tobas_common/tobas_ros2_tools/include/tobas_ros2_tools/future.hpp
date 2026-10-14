// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <future>

namespace tobas::ros2
{
/** Similar to `future.wait_for()`, but waits indefinitely when the timeout is non-positive. */
template <typename FutureType, typename RepType, typename DurType>
std::future_status waitForFuture(const FutureType& _future, std::chrono::duration<RepType, DurType> _timeout)
{
  if (_timeout.count() > 0) {
    return _future.wait_for(_timeout);
  }
  else {
    _future.wait();
    return std::future_status::ready;
  }
}
}  // namespace tobas::ros2
