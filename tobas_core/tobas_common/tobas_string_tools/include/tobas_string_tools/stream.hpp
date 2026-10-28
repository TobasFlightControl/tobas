// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

namespace tobas::str
{
std::expected<std::string, std::string> readText(const std::string& _path);

std::expected<void, std::string> writeText(const std::string& _path, const std::string& _text);
}  // namespace tobas::str
