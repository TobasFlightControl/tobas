// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

namespace tobas
{
namespace manipulation
{
static constexpr auto kAutoResetTimeThresh = std::chrono::milliseconds(500);
}  // namespace manipulation
}  // namespace tobas
