// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/base_controller.hpp"

#include <tobas_path_tools/join.hpp>

namespace tobas
{
namespace rc
{
BaseController::BaseController()
{
  // Do not access ROS parameters or register publishers and subscriptions here because they may be unnecessary.
}

std::string BaseController::addMode(const std::string& text, FlightMode mode)
{
  return path::join(textFromEnum(mode), text);
}
}  // namespace rc
}  // namespace tobas
