#include "tobas_ros2_tools/util.hpp"

#include <string.h>

#include <iostream>

#include <rcutils/env.h>
#include <rcutils/filesystem.h>

namespace fs = std::filesystem;

namespace ros2
{
const char* getEnv(const char* name)
{
  const char* value = nullptr;
  const char* error = rcutils_get_env(name, &value);

  if (error) {
    std::cerr << "Failed to get \"" << name << "\": " << error << std::endl;
    return nullptr;
  }

  if (strlen(value) == 0) {
    std::cerr << "\"" << name << "\" is not set." << std::endl;
    return nullptr;
  }

  return value;
}

const char* getUserName()
{
  return getEnv("USER");
}

fs::path expandUser(const char* path)
{
  return rcutils_expand_user(path, rcutils_get_default_allocator());
}
}  // namespace ros2
