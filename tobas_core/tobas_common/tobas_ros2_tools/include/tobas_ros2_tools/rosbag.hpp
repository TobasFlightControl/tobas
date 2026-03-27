#pragma once

#include <string>

namespace tobas
{
namespace ros2
{
bool reindexRosBag(const std::string& uri, const std::string& storage_id = "mcap") noexcept;
}  // namespace ros2
}  // namespace tobas
