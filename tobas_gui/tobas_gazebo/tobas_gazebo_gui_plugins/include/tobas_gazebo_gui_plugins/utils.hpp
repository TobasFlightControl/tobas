// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

namespace tobas
{
namespace gazebo
{
std::expected<std::string, std::string> getWorldName();
}  // namespace gazebo
}  // namespace tobas
