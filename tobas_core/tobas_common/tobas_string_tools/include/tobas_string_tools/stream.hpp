// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace str
{
bool readText(const std::string& path, std::string& text);

bool writeText(const std::string& path, const std::string& text);
}  // namespace str
}  // namespace tobas
