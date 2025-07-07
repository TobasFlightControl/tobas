#pragma once

#include <string>

namespace ros2
{
/* XACROを展開してURDFのstringとして返す．URDFをそのまま渡しても問題ない． */
bool parseXacroFromPath(const std::string& xacro_path, std::string& urdf_text);

/* XACROを展開してURDFのstringとして返す．URDFをそのまま渡しても問題ない． */
bool parseXacroFromText(const std::string& xacro_text, std::string& urdf_text);
}  // namespace ros2
