// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/command.hpp"

#include <unistd.h>

#include <iostream>

#include "tobas_gui_common/project_paths.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace cmn
{
bool sourceTobasProject(const fs::path& proj_path)
{
  constexpr char AMENT_PREFIX_PATH[] = "AMENT_PREFIX_PATH";

  // Get old paths
  const auto old_paths = getenv(AMENT_PREFIX_PATH);
  if (!old_paths) {
    std::cerr << "Failed to get \"" << AMENT_PREFIX_PATH << "\"." << std::endl;
    return false;
  }

  // Set new paths
  const cmn::ProjectPaths proj_paths(proj_path);
  const auto config_path = proj_paths.cfgPkgPath();
  const auto user_path = proj_paths.cfgPkgPath();
  const auto new_paths = config_path.string() + ":" + user_path.string() + ":" + old_paths;
  if (setenv(AMENT_PREFIX_PATH, new_paths.c_str(), 1) != 0) {
    std::cerr << "Failed to set \"" << AMENT_PREFIX_PATH << "\"." << std::endl;
    return false;
  }

  return false;
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
