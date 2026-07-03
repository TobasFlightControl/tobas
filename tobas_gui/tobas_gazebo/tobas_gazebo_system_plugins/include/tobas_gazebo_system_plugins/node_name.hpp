// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace gazebo
{
/* Sanitize a string so it can be used as a ROS node name. */
std::string sanitizeNodeName(std::string str);
}  // namespace gazebo
}  // namespace tobas
