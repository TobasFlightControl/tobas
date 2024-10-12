#pragma once

#include <filesystem>

namespace ros2
{
std::filesystem::path expandUser(const char* path);
}
