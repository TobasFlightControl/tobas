// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

namespace tobas
{
namespace ros2
{
std::expected<void, std::string> reindexRosBag(const std::string& uri, const std::string& storage_id = "mcap") noexcept;
}  // namespace ros2
}  // namespace tobas
