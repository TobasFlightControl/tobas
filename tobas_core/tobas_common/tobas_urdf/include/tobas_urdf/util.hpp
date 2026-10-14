// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <filesystem>
#include <string>

namespace tobas::urdf
{
/** Return the absolute path of a file in a URDF. */
std::expected<std::filesystem::path, std::string> resolveUri(const std::string& _uri);
}  // namespace tobas::urdf
