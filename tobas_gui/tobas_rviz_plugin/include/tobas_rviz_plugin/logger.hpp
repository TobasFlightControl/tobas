// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/logger.hpp>

namespace tobas
{
/* Creates a namespaced logger. */
rclcpp::Logger getLogger(const std::string& name);
}  // namespace tobas
