#pragma once

#include <filesystem>

namespace tobas
{
bool buildTobasPackage(const std::filesystem::path& tbs_path);
bool sourceTobasPackage(const std::filesystem::path& tbs_path);
}  // namespace tobas
