#pragma once

#include <filesystem>

namespace ros2
{
const char* getEnv(const char* name);

const char* getUserName();

std::filesystem::path expandUser(const char* path);
}  // namespace ros2
