// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace tobas::gui::gcs
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory("tobas_gcs");
}

fs::path getResourceDir()
{
  return getPkgShareDir() / "resources";
}
}  // namespace tobas::gui::gcs
