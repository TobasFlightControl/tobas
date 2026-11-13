// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

namespace tobas::hardware
{
static constexpr auto kRetryInitializationInterval = std::chrono::seconds(1);
}  // namespace tobas::hardware
