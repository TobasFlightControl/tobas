// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/time.hpp>

namespace tobas::ros2
{
class RateManager
{
public:
  explicit RateManager(double _update_rate);

  void reset();

  /** Return true when the period is executable. */
  bool update(const rclcpp::Time& _time);

private:
  const rclcpp::Duration interval_;
  rclcpp::Time t_next_;
  bool is_first_update_ = true;
};
}  // namespace tobas::ros2
