// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace ros2
{
bool reindexRosBag(const std::string& _uri, const std::string& _storage_id = "mcap") noexcept;
}  // namespace ros2
}  // namespace tobas
