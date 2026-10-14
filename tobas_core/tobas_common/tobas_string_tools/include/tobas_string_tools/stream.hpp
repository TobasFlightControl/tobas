// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas::str
{
bool readText(const std::string& _path, std::string& _text);

bool writeText(const std::string& _path, const std::string& _text);
}  // namespace tobas::str
