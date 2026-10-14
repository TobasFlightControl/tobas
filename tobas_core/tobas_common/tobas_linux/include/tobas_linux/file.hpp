// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas::linux
{
int writeFile(const char* _path, const char* _fmt, ...);
int readFile(const char* _path, const char* _fmt, ...);
}  // namespace tobas::linux
