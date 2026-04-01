// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

namespace tobas
{
// Common period & timeout
static constexpr auto kCheckTopicsPeriod = std::chrono::seconds(5);
static constexpr auto kCommandAutoResetTimeout = std::chrono::milliseconds(500);

// Console message period
static constexpr double kTypicalInfoPeriod = 5.;   // [s]
static constexpr double kTypicalWarnPeriod = 3.;   // [s]
static constexpr double kTypicalErrorPeriod = 1.;  // [s]
static constexpr double kIgnoreCmdMsgPeriod = 1.;  // [s]
}  // namespace tobas
