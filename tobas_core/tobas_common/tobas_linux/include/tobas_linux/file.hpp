// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace linux
{
int writeFile(const char* path, const char* fmt, ...);
int readFile(const char* path, const char* fmt, ...);
}  // namespace linux
}  // namespace tobas
