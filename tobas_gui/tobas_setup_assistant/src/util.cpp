// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tobas_setup_assistant/constants.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace sa
{
fs::path getPkgShareDir()
{
  return ament_index_cpp::get_package_share_directory("tobas_setup_assistant");
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
