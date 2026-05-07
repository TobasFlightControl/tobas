// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

namespace tobas
{
namespace gui
{
namespace log
{
static constexpr auto kRecordServiceTimeout = std::chrono::seconds(5);
}  // namespace log
}  // namespace gui
}  // namespace tobas
