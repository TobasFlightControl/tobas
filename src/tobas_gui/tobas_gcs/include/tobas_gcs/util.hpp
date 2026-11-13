// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

namespace tobas::gui::gcs
{
std::filesystem::path getPkgShareDir();
std::filesystem::path getResourceDir();
}  // namespace tobas::gui::gcs
