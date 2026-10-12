// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/command_priority_handler.hpp"

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

namespace tobas
{
CommandPriorityHandler::CommandPriorityHandler()
{
}

bool CommandPriorityHandler::update(uint8_t _new_priority, const rclcpp::Time& _cur_time)
{
  constexpr auto kHighestLevelTimeout = 500ms;

  if (_new_priority >= cur_priority_ || _cur_time - t_last_highest_priority_ > kHighestLevelTimeout) {
    if (_new_priority > cur_priority_) {
      std::cout << "Command priority up: " << (int)cur_priority_ << " -> " << (int)_new_priority << std::endl;
    }
    else if (_new_priority < cur_priority_) {
      std::cout << "Command priority down: " << (int)cur_priority_ << " -> " << (int)_new_priority << std::endl;
    }

    cur_priority_ = _new_priority;
    t_last_highest_priority_ = _cur_time;
    return true;
  }
  else {
    return false;
  }
}
}  // namespace tobas
