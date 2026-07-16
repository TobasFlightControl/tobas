// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cinttypes>

namespace tobas
{
namespace st
{
/* Calculate GPS delay from the given GPS ToW (GPS Time of Week) and UTC obtained from an NTP server. */
long computeGpsDelayFromToW(uint32_t gps_tow_ms);
}  // namespace st
}  // namespace tobas
