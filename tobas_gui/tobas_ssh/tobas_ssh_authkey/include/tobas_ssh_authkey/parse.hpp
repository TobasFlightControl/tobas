// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <filesystem>
#include <vector>

#include "./data.hpp"

namespace tobas
{
namespace ssh
{
namespace ak
{
std::expected<Data, std::string> parseLine(const std::string& line);

std::expected<std::vector<Data>, std::string> parseFile(const std::filesystem::path& path);
}  // namespace ak
}  // namespace ssh
}  // namespace tobas
