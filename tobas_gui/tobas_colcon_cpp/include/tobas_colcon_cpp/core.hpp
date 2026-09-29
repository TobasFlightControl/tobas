// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <filesystem>
#include <string>

namespace tobas
{
namespace colcon
{
class Colcon
{
public:
  explicit Colcon();

  std::expected<void, std::string> build(const std::filesystem::path& pkg_path, const std::filesystem::path& ws_path);

  std::expected<void, std::string> cleanWorkspace(const std::filesystem::path& ws_path);

  void setParallelWorkers(size_t num);
  void setMergeInstall(bool enabled);
  void setSymlinkInstall(bool enabled);
  void setCmakeCleanCache(bool enabled);

private:
  struct BuildOptions
  {
    size_t parallel_workers = 0;
    bool merge_install = false;
    bool symlink_install = false;
    bool cmake_clean_cache = false;
  } build_opts_;

  static std::filesystem::path buildBase(const std::filesystem::path& ws_path);
  static std::filesystem::path installBase(const std::filesystem::path& ws_path);
  static std::filesystem::path logBase(const std::filesystem::path& ws_path);
};
}  // namespace colcon
}  // namespace tobas
