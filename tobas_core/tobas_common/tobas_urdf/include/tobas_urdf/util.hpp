#pragma once

#include <filesystem>

namespace urdf
{
/* URDF中のファイルの絶対パスを返す． */
std::filesystem::path resolveURI(const std::string& uri);
}  // namespace urdf
