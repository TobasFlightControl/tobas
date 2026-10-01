// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <exception>
#include <expected>
#include <filesystem>
#include <string>
#include <vector>

#include <rapidcsv.h>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace csv
{
rapidcsv::Document load(const std::filesystem::path& path);

template <typename T>
std::expected<std::vector<T>, std::string> getColumn(const rapidcsv::Document& doc, const std::string& name)
{
  try {
    return doc.GetColumn<T>(name);
  }
  catch (const std::exception& e) {
    return std::unexpected(std::string(e.what()));
  }
}
}  // namespace csv
}  // namespace sa
}  // namespace gui
}  // namespace tobas
