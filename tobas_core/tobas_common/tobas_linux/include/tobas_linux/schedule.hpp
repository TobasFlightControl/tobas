// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace linux
{
bool setRealtimePriorityFIFO(int priority);
bool setRealtimePriorityRR(int priority);
}  // namespace linux
}  // namespace tobas
