// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>
#include <expected>
#include <filesystem>
#include <string>

namespace tobas
{
namespace colcon
{
struct BuildOptions
{
  size_t parallel_workers = 0;  ///< Zero uses all available processors.
  bool merge_install = false;
  bool symlink_install = false;
  bool cmake_clean_cache = false;
};

std::expected<void, std::string>
build(const std::filesystem::path& pkg_path, const std::filesystem::path& ws_path, const BuildOptions& options = {});

std::expected<void, std::string> cleanWorkspace(const std::filesystem::path& ws_path);
}  // namespace colcon
}  // namespace tobas
