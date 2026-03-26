#pragma once

#include <filesystem>

namespace tobas
{
namespace ros2
{
const char* getEnv(const char* name);

const char* getUserName();

const char* getHomeDir();

std::filesystem::path expandUser(const char* path);
}  // namespace ros2
}  // namespace tobas
