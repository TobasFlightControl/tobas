// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace linux
{
bool setRealtimePriorityFIFO(const int& priority);
bool setRealtimePriorityRR(const int& priority);
}  // namespace linux
}  // namespace tobas
