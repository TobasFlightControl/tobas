#pragma once

#include <string>

namespace ros2
{
/* XACROを展開してURDFのstringとして返す．URDFをそのまま渡しても問題ない． */
bool xacro(const std::string& xacro_path, std::string& urdf_content);
}  // namespace ros2
