// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/util.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>

namespace fs = std::filesystem;

namespace tobas::gui::sa
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_path("tobas_setup_assistant");
}
}  // namespace tobas::gui::sa
