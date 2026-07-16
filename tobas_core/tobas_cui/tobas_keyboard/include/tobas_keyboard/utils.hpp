// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cinttypes>
#include <expected>

namespace tobas
{
namespace keyboard
{
/* Get keyboard repeat interval [ms]. */
std::expected<uint16_t, const char*> getKeyboardRepeatInterval();
}  // namespace keyboard
}  // namespace tobas
