#pragma once

#include <expected>
#include <string>

namespace gazebo
{
std::expected<std::string, std::string> getWorldName();
}  // namespace gazebo
