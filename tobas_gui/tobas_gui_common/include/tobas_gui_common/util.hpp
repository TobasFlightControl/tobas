#pragma once

#include <filesystem>

namespace gui
{
namespace common
{
std::filesystem::path getIconPath();

bool addAmentPrefixPath(const std::filesystem::path& path);
}  // namespace common
}  // namespace gui
