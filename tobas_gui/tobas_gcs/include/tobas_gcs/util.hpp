#pragma once

#include <filesystem>

namespace gui
{
namespace gcs
{
std::filesystem::path getPkgShareDir();
std::filesystem::path getResourceDir();
}  // namespace gcs
}  // namespace gui
