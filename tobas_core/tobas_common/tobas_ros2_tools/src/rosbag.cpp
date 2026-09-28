// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/rosbag.hpp"

#include <rosbag2_cpp/reindexer.hpp>

namespace tobas
{
namespace ros2
{
std::expected<void, std::string> reindexRosBag(const std::string& uri, const std::string& storage_id) noexcept
{
  rosbag2_cpp::Reindexer reindexer;

  rosbag2_storage::StorageOptions options;
  options.uri = uri;
  options.storage_id = storage_id;

  try {
    reindexer.reindex(options);
  }
  catch (const std::exception& e) {
    return std::unexpected("Failed to reindex " + uri + ": " + e.what());
  }

  return {};
}
}  // namespace ros2
}  // namespace tobas
