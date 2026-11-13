// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <exception>
#include <expected>
#include <filesystem>
#include <string>
#include <vector>

#include <rapidcsv.h>

namespace tobas::gui::sa::csv
{
rapidcsv::Document load(const std::filesystem::path& _path);

template <typename T>
std::expected<std::vector<T>, std::string> getColumn(const rapidcsv::Document& _doc, const std::string& _name)
{
  try {
    return _doc.GetColumn<T>(_name);
  }
  catch (const std::exception& e) {
    return std::unexpected(std::string(e.what()));
  }
}
}  // namespace tobas::gui::sa::csv
