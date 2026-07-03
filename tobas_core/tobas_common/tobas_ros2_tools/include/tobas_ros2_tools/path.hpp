// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace ros2
{
/**
 * @brief Create a temporary file.
 *
 * @param path Path of the created file.
 * @return File descriptor.
 */
int createTemporalFile(std::string& path);
}  // namespace ros2
}  // namespace tobas
