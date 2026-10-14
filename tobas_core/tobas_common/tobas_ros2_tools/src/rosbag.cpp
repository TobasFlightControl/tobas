// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/rosbag.hpp"

#include <iostream>

#include <rosbag2_cpp/reindexer.hpp>

namespace tobas
{
namespace ros2
{
bool reindexRosBag(const std::string& _uri, const std::string& _storage_id) noexcept
{
  rosbag2_cpp::Reindexer reindexer;

  rosbag2_storage::StorageOptions options;
  options.uri = _uri;
  options.storage_id = _storage_id;

  try {
    reindexer.reindex(options);
  }
  catch (const std::exception& e) {
    std::cerr << "Failed to reindex " << _uri + ": " << e.what() << '\n';
    return false;
  }

  return true;
}
}  // namespace ros2
}  // namespace tobas
