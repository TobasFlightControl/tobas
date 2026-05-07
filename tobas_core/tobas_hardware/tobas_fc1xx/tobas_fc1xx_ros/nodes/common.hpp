// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

namespace tobas
{
namespace fc1xx
{
static constexpr auto kRetryInitializationInterval = std::chrono::seconds(1);
}  // namespace fc1xx
}  // namespace tobas
