#pragma once

#include <filesystem>

namespace ros2
{
/* URDF中のファイルの絶対パスを返す． */
std::filesystem::path resolveURI(const std::string& uri);
}  // namespace ros2
