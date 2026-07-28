// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

namespace tobas
{
// Common period & timeout
static constexpr auto kCheckTopicsPeriod = std::chrono::seconds(5);
static constexpr auto kCommandAutoResetTimeout = std::chrono::milliseconds(500);

// Console message period
static constexpr double kTypicalInfoPeriod = 5.0;   // [s]
static constexpr double kTypicalWarnPeriod = 3.0;   // [s]
static constexpr double kTypicalErrorPeriod = 1.0;  // [s]
static constexpr double kIgnoreCmdMsgPeriod = 1.0;  // [s]
}  // namespace tobas
