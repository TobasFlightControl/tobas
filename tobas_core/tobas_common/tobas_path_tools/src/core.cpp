// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_path_tools/core.hpp"

#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

namespace tobas
{
namespace path
{
bool isReadable(const fs::path& file_path)
{
  const std::ifstream ifs(file_path);
  return ifs.good();
}

bool isWritable(const fs::path& file_path)
{
  const std::ofstream ofs(file_path, std::ios::app);  // Open in append mode to avoid destroying existing files.
  return ofs.good();
}

std::expected<void, std::string> createDirectories(const fs::path& dir_path, bool exist_ok)
{
  if (fs::is_directory(dir_path)) {
    if (exist_ok) {
      return {};
    }
    else {
      return std::unexpected("\"" + dir_path.string() + "\" already exists.");
    }
  }

  std::error_code ec;
  if (!fs::create_directories(dir_path, ec)) {
    return std::unexpected(ec.message());
  }

  return {};
}

std::expected<void, std::string> createFilePath(const fs::path& file_path, bool exist_ok)
{
  // Check whether the file exists.
  if (fs::is_regular_file(file_path)) {
    if (exist_ok) {
      return {};
    }
    else {
      return std::unexpected("\"" + file_path.string() + "\" already exists.");
    }
  }

  // Get only the directory part from the file path.
  const auto dir_path = fs::path(file_path).parent_path();

  // Create the path up to the parent directory of the file.
  const auto create_dir_res = createDirectories(dir_path, true);
  if (!create_dir_res) {
    return std::unexpected(create_dir_res.error());
  }

  // Create an empty file.
  const std::ofstream file(file_path);
  if (!file) {
    return std::unexpected("Failed to create \"" + file_path.string() + "\".");
  }

  return {};
}

size_t computeDirectorySize(const fs::path& dir_path)
{
  if (!fs::is_directory(dir_path)) {
    std::cerr << dir_path << " does not exist." << std::endl;
    return 0;
  }

  size_t total_size = 0;

  // Recursively scan the directory contents.
  for (const auto& entry : fs::recursive_directory_iterator(dir_path)) {
    if (fs::is_regular_file(entry.status())) {
      total_size += fs::file_size(entry.path());
    }
  }

  return total_size;
}

std::expected<void, std::string> clearDirectory(const fs::path& dir_path)
{
  if (!fs::is_directory(dir_path)) {
    return std::unexpected("\"" + dir_path.string() + "\" does not exist.");
  }

  for (const auto& entry : fs::directory_iterator(dir_path)) {
    fs::remove_all(entry);
  }

  return {};
}
}  // namespace path
}  // namespace tobas
