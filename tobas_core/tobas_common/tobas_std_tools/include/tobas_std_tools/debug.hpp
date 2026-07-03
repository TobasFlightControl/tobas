// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

/* Print the file name and line number of the call site. */
#define PRINT_LOCATION() st::_printLocation(__FILE__, __LINE__)

namespace tobas
{
namespace st
{
void _printLocation(const char* file, int line);
}  // namespace st
}  // namespace tobas
