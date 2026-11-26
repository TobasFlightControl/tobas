// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ntrip_client/source_table_reader.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <sstream>
#include <utility>

#include <tobas_std_tools/universal_constants.hpp>

namespace tobas::ntrip
{
SourceTableReader::SourceTableReader(const std::string& _data)
{
  std::stringstream data_stream(_data);
  std::string record;
  while (std::getline(data_stream, record)) {
    // Keep only the stream records. Lines such as `SOURCETABLE 200 OK` do not describe a mount point.
    if (!record.starts_with("STR")) {
      continue;
    }

    std::vector<std::string> columns;
    std::stringstream record_stream(record);
    std::string column;
    while (std::getline(record_stream, column, ';')) {
      columns.push_back(column);
    }

    // Keep the record only if it reaches at least the longitude column.
    if (columns.size() > kLongitude) {
      mount_points_.push_back(columns);
    }
  }
  std::cout << "Read " << mount_points_.size() << " mount points." << std::endl;
}

std::vector<std::string> SourceTableReader::sortMountPoints(double _latitude, double _longitude)
{
  // Project the query position onto the unit sphere.
  const auto query_x = std::cos(_latitude * st::kDeg2Rad) * std::cos(_longitude * st::kDeg2Rad);
  const auto query_y = std::cos(_latitude * st::kDeg2Rad) * std::sin(_longitude * st::kDeg2Rad);
  const auto query_z = std::sin(_latitude * st::kDeg2Rad);

  // Compute the squared chord distance from the query position to every mount point.
  std::vector<std::pair<double, std::string>> distances_and_names;
  distances_and_names.reserve(mount_points_.size());
  for (const auto& columns : mount_points_) {
    double mount_point_latitude = 0.0;
    double mount_point_longitude = 0.0;
    try {
      mount_point_latitude = std::stod(columns[kLatitude]);
    }
    catch (const std::exception& e) {
      std::cerr << "Failed to read the latitude of a mount point: " << e.what() << std::endl;
    }
    try {
      mount_point_longitude = std::stod(columns[kLongitude]);
    }
    catch (const std::exception& e) {
      std::cerr << "Failed to read the longitude of a mount point: " << e.what() << std::endl;
    }

    // Project the mount point onto the unit sphere as well.
    const auto mount_point_x =
      std::cos(mount_point_latitude * st::kDeg2Rad) * std::cos(mount_point_longitude * st::kDeg2Rad);
    const auto mount_point_y =
      std::cos(mount_point_latitude * st::kDeg2Rad) * std::sin(mount_point_longitude * st::kDeg2Rad);
    const auto mount_point_z = std::sin(mount_point_latitude * st::kDeg2Rad);

    const auto dx = mount_point_x - query_x;
    const auto dy = mount_point_y - query_y;
    const auto dz = mount_point_z - query_z;
    distances_and_names.emplace_back(dx * dx + dy * dy + dz * dz, columns[kMountPoint]);
  }

  // Sort by ascending distance and return the names only.
  std::sort(distances_and_names.begin(), distances_and_names.end());

  std::vector<std::string> sorted_names;
  sorted_names.reserve(distances_and_names.size());
  for (const auto& distance_and_name : distances_and_names) {
    sorted_names.push_back(distance_and_name.second);
  }
  return sorted_names;
}
}  // namespace tobas::ntrip
