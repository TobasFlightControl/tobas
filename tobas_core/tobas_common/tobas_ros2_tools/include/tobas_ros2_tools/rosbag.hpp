#pragma once

#include <string>

namespace ros2
{
bool reindexRosBag(const std::string& uri, const std::string& storage_id = "mcap") noexcept;
}  // namespace ros2
