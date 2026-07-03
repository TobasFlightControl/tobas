// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <mutex>

#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>

namespace tobas
{
namespace gazebo
{
/* Receive exactly one Gazebo message. */
template <typename MsgT, typename RepT = int64_t, typename RatioT = std::milli>
bool waitForMessage(
  MsgT& _msg_out,
  const std::string& _topic,
  std::chrono::duration<RepT, RatioT> _timeout = std::chrono::duration<RepT, RatioT>(-1))
{
  gz::transport::Node node;
  std::mutex mutex;
  std::condition_variable cv;
  bool got = false;

  // Callback; only the first message is accepted.
  const std::function<void(const MsgT&)> cb = [&](const MsgT& msg)
  {
    const std::lock_guard lock(mutex);
    if (got) {
      return;  // Ignore the second and later messages.
    }
    _msg_out = msg;
    got = true;
    cv.notify_one();
  };

  // Start subscription.
  if (!node.Subscribe(_topic, cb)) {
    gzerr << "Failed to subscribe \"" << _topic << "\"." << std::endl;
    return false;
  }

  // Wait for message reception.
  {
    std::unique_lock<std::mutex> lock(mutex);
    const auto wait_cb = [&] { return got; };
    if (_timeout.count() > 0) {
      cv.wait_for(lock, _timeout, wait_cb);
    }
    else {
      cv.wait(lock, wait_cb);
    }
  }

  // Unsubscribe.
  node.Unsubscribe(_topic);

  return got;
}
}  // namespace gazebo
}  // namespace tobas
