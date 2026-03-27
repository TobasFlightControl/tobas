#pragma once

#include <expected>
#include <string>

namespace tobas
{
namespace gazebo
{
std::expected<std::string, std::string> getWorldName();
}  // namespace gazebo
}  // namespace tobas
