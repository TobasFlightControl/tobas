#pragma once

#include <string>

namespace gazebo
{
/* 文字列をROSノード名に使用可能なものに修正する． */
std::string sanitizeNodeName(std::string str);
}  // namespace gazebo
