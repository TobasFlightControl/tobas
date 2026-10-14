// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace tobas::gui::ub
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory("tobas_urdf_builder");
}
}  // namespace tobas::gui::ub
