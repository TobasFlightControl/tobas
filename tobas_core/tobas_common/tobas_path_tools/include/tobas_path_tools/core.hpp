// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <filesystem>

namespace tobas
{
namespace path
{
/* Return true when the file is readable. */
bool isReadable(const std::filesystem::path& file_path);

/* Return true when the file is writable. */
bool isWritable(const std::filesystem::path& file_path);

/**
 * @brief Create a directory, including intermediate paths.
 * Unlike `fs::create_directories`, this returns true for an existing directory when `exist_ok = true`.
 */
std::expected<void, std::string> createDirectories(const std::filesystem::path& dir_path, bool exist_ok = true);

/* Create a file and intermediate paths. */
std::expected<void, std::string> createFilePath(const std::filesystem::path& file_path, bool exist_ok = true);

/* Calculate the total size [bytes] of all files in a directory. */
size_t computeDirectorySize(const std::filesystem::path& dir_path);

/* Remove all files and subdirectories in a directory. */
std::expected<void, std::string> clearDirectory(const std::filesystem::path& dir_path);
}  // namespace path
}  // namespace tobas
