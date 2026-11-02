// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/path.hpp"

#include <filesystem>

namespace fs = std::filesystem;

namespace tobas::ros2
{
int createTemporalFile(std::string& _path)
{
  // Get the system temporary directory.
  const auto tmp_dir = fs::temp_directory_path();

  // Create the template string. The last six characters must be X.
  _path = (tmp_dir / "tobas_temporal_file_XXXXXX").string();

  // `mkstemp()` replaces the X part of the template with random characters and creates a temporary file.
  return mkstemp(_path.data());
}
}  // namespace tobas::ros2
