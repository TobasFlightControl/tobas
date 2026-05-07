// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace st
{
bool commandLineOptionExists(char** begin, char** end, const char* option);
char* getCommandLineOption(char** begin, char** end, const char* option);
}  // namespace st
}  // namespace tobas
