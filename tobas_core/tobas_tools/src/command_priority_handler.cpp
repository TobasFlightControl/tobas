// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/command_priority_handler.hpp"

#include <iostream>

using namespace std;

namespace tobas
{
CommandPriorityHandler::CommandPriorityHandler()
{
}

bool CommandPriorityHandler::update(const uint8_t& new_priority, const rclcpp::Time& cur_time)
{
  if (new_priority >= cur_priority_ || cur_time - t_last_highest_priority_ > kHighestLevelTimeout) {
    if (new_priority > cur_priority_) {
      cout << "Command priority up: " << (int)cur_priority_ << " -> " << (int)new_priority << endl;
    }
    else if (new_priority < cur_priority_) {
      cout << "Command priority down: " << (int)cur_priority_ << " -> " << (int)new_priority << endl;
    }

    cur_priority_ = new_priority;
    t_last_highest_priority_ = cur_time;
    return true;
  }
  else {
    return false;
  }
}
}  // namespace tobas
