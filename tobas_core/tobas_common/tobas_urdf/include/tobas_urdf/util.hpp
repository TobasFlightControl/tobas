// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

namespace tobas
{
namespace urdf
{
/* Return the absolute path of a file in a URDF. */
std::filesystem::path resolveUri(const std::string& uri);
}  // namespace urdf
}  // namespace tobas
