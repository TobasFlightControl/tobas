// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace bm
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory("tobas_bootmedia_config");
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
