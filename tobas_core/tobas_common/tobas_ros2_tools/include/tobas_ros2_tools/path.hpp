// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace ros2
{
/**
 * Create a temporary file.
 *
 * @param _path Path of the created file.
 * @return File descriptor.
 */
int createTemporalFile(std::string& _path);
}  // namespace ros2
}  // namespace tobas
