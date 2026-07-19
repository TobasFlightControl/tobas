// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_components_rt/timer_coalescing_events_queue.hpp"

namespace ch = std::chrono;

namespace tobas
{
void TimerCoalescingEventsQueue::enqueue(const ExecutorEvent& event)
{
  bool event_added = false;

  {
    const std::lock_guard lock(mutex_);

    auto single_event = event;
    single_event.num_events = 1;

    switch (event.type) {
      case rclcpp::experimental::executors::TIMER_EVENT:
        if (event.num_events > 0) {
          auto& is_pending = timer_pending_states_[event.entity_key];
          if (!is_pending) {
            event_queue_.push(single_event);
            is_pending = true;
            event_added = true;
          }
        }
        break;
      default:
        for (size_t _ = 0; _ < event.num_events; ++_) {
          event_queue_.push(single_event);
          event_added = true;
        }
        break;
    }
  }

  if (event_added) {
    events_queue_cv_.notify_one();
  }
}

bool TimerCoalescingEventsQueue::dequeue(ExecutorEvent& event, ch::nanoseconds timeout)
{
  std::unique_lock lock(mutex_);

  bool has_data = true;
  if (timeout == ch::nanoseconds::max()) {
    events_queue_cv_.wait(lock, [this]() { return !event_queue_.empty(); });
  }
  else {
    has_data = events_queue_cv_.wait_for(lock, timeout, [this]() { return !event_queue_.empty(); });
  }

  if (!has_data) {
    return false;
  }

  event = event_queue_.front();
  event_queue_.pop();

  if (event.type == rclcpp::experimental::executors::TIMER_EVENT) {
    const auto it = timer_pending_states_.find(event.entity_key);
    if (it != timer_pending_states_.end()) {
      // Keep the entry to avoid allocating memory again on every timer period.
      it->second = false;
    }
  }

  return true;
}

bool TimerCoalescingEventsQueue::empty() const
{
  const std::lock_guard lock(mutex_);
  return event_queue_.empty();
}

size_t TimerCoalescingEventsQueue::size() const
{
  const std::lock_guard lock(mutex_);
  return event_queue_.size();
}
}  // namespace tobas
