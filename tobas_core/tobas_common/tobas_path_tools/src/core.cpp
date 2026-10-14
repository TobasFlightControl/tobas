// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_path_tools/core.hpp"

#include <fstream>

namespace fs = std::filesystem;

namespace tobas
{
namespace path
{
bool isReadable(const fs::path& _file_path)
{
  const std::ifstream ifs(_file_path);
  return ifs.good();
}

bool isWritable(const fs::path& _file_path)
{
  const std::ofstream ofs(_file_path, std::ios::app);  // Open in append mode to avoid destroying existing files.
  return ofs.good();
}

std::expected<void, std::string> createDirectories(const fs::path& _dir_path, bool _exist_ok)
{
  if (fs::is_directory(_dir_path)) {
    if (_exist_ok) {
      return {};
    }
    else {
      return std::unexpected("'" + _dir_path.string() + "' already exists.");
    }
  }

  std::error_code ec;
  if (!fs::create_directories(_dir_path, ec)) {
    return std::unexpected(ec.message());
  }

  return {};
}

std::expected<void, std::string> createFilePath(const fs::path& _file_path, bool _exist_ok)
{
  // Check whether the file exists.
  if (fs::is_regular_file(_file_path)) {
    if (_exist_ok) {
      return {};
    }
    else {
      return std::unexpected("'" + _file_path.string() + "' already exists.");
    }
  }

  // Get only the directory part from the file path.
  const auto dir_path = fs::path(_file_path).parent_path();

  // Create the path up to the parent directory of the file.
  const auto create_dir_res = createDirectories(dir_path, true);
  if (!create_dir_res) {
    return std::unexpected(create_dir_res.error());
  }

  // Create an empty file.
  const std::ofstream file(_file_path);
  if (!file) {
    return std::unexpected("Failed to create '" + _file_path.string() + "'.");
  }

  return {};
}

std::expected<size_t, std::string> computeDirectorySize(const fs::path& _dir_path)
{
  if (!fs::is_directory(_dir_path)) {
    return std::unexpected("'" + _dir_path.string() + "' is not a directory.");
  }

  size_t total_size = 0;

  // Recursively scan the directory contents.
  for (const auto& entry : fs::recursive_directory_iterator(_dir_path)) {
    if (fs::is_regular_file(entry.status())) {
      total_size += fs::file_size(entry.path());
    }
  }

  return total_size;
}

std::expected<void, std::string> clearDirectory(const fs::path& _dir_path)
{
  if (!fs::is_directory(_dir_path)) {
    return std::unexpected("'" + _dir_path.string() + "' does not exist.");
  }

  for (const auto& entry : fs::directory_iterator(_dir_path)) {
    fs::remove_all(entry);
  }

  return {};
}
}  // namespace path
}  // namespace tobas
