// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/base_controller.hpp"

#include <cassert>

#include <tobas_path_tools/join.hpp>

namespace tobas::rc
{
BaseController::BaseController()
{
  // Do not access ROS parameters or register publishers and subscriptions here because they may be unnecessary.
}

void BaseController::setDeadband(double deadband)
{
  assert(0.0 <= deadband && deadband <= 1.0);
  deadband_ = deadband;
}

std::string BaseController::addMode(const std::string& _text, FlightMode _mode)
{
  return path::join(textFromEnum(_mode), _text);
}
}  // namespace tobas::rc
