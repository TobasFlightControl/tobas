// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

namespace tobas
{
namespace hardware
{
static constexpr auto kRetryInitializationInterval = std::chrono::seconds(3);
}  // namespace hardware
}  // namespace tobas
