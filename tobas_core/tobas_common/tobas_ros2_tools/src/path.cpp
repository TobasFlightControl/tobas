// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/path.hpp"

#include <rcpputils/filesystem_helper.hpp>

namespace rfs = rcpputils::fs;

namespace tobas
{
namespace ros2
{
int createTemporalFile(std::string& path)
{
  // Get the system temporary directory.
  const auto tmp_dir = rfs::temp_directory_path();

  // Create the template string. The last six characters must be X.
  path = (tmp_dir / "tobas_temporal_file_XXXXXX").string();

  // `mkstemp()` replaces the X part of the template with random characters and creates a temporary file.
  return mkstemp(path.data());
}
}  // namespace ros2
}  // namespace tobas
