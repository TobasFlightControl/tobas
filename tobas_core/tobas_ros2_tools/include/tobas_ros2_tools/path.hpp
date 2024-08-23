#pragma once

#include <string>
#include <vector>

namespace ros2
{
/* URDF中のファイルの絶対パスを返す． */
std::string resolveURI(const std::string& uri);
}  // namespace ros2
