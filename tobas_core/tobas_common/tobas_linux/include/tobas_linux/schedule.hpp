// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas::linux
{
bool setRealtimePriorityFIFO(int _priority);
bool setRealtimePriorityRR(int _priority);
}  // namespace tobas::linux
