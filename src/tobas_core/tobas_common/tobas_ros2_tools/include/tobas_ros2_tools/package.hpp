// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <filesystem>

namespace tobas::ros2
{
/** Return the path of the package that `path` belongs to. */
std::expected<std::filesystem::path, std::string> getPackagePathOf(const std::filesystem::path& _path);

/** Read the `name` in `package.xml` of the package that `path` belongs to. */
std::expected<std::string, std::string> getPackageNameOf(const std::filesystem::path& _path);

/** Return the parent directory of the nearest `src` directory from the package path that `path` belongs to. */
std::expected<std::filesystem::path, std::string> estimateWorkspaceOf(const std::filesystem::path& _path);

/** Check whether the package has already been built and installed. */
bool isAlreadyBuiltAndInstalled(const std::filesystem::path& _pkg_path);

/** Check whether the package name is valid. */
bool isValidPackageName(const std::string& _pkg_name);
}  // namespace tobas::ros2
