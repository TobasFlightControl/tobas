// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <cstdint>

#include <rclcpp/time.hpp>

namespace tobas
{
class CommandPriorityHandler
{
  static constexpr auto kHighestLevelTimeout = std::chrono::milliseconds(500);

public:
  explicit CommandPriorityHandler();

  /**
   * @brief Receive a command priority and determine whether to accept it.
   *
   * Conditions for accepting a command:
   * 1. The priority is greater than or equal to the current priority.
   * 2. A fixed time has passed since the last highest-priority command was received.
   *
   * @param new_priority Received command priority.
   * @param cur_time Current time.
   * @return true if the command is accepted.
   * @return false if the command is not accepted.
   */
  bool update(const uint8_t& new_priority, const rclcpp::Time& cur_time);

private:
  uint8_t cur_priority_ = 0;
  rclcpp::Time t_last_highest_priority_;  // Last time a highest-priority command arrived
};
}  // namespace tobas
