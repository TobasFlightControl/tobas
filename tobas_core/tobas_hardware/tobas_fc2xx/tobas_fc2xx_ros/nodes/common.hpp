// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

namespace tobas
{
namespace fc2xx
{
static constexpr auto kRetryInitializationInterval = std::chrono::seconds(3);
}  // namespace fc2xx
}  // namespace tobas
