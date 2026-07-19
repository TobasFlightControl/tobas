// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <unordered_map>

#include <rclcpp/experimental/executors/events_executor/events_queue.hpp>

namespace tobas
{
/** @brief An events queue that keeps at most one pending event for each timer. */
class TimerCoalescingEventsQueue : public rclcpp::experimental::executors::EventsQueue
{
public:
  using ExecutorEvent = rclcpp::experimental::executors::ExecutorEvent;

  void enqueue(const ExecutorEvent& event) override;
  bool dequeue(ExecutorEvent& event, std::chrono::nanoseconds timeout) override;
  bool empty() const override;
  size_t size() const override;

private:
  std::queue<ExecutorEvent> event_queue_;
  std::unordered_map<const void*, bool> timer_pending_states_;
  mutable std::mutex mutex_;
  std::condition_variable events_queue_cv_;
};
}  // namespace tobas
