#include <rcutils/filesystem.h>

#include "../include/tobas_ros2_tools/filesystem.hpp"

using namespace std;
namespace fs = filesystem;

namespace ros2
{
fs::path expandUser(const char* path)
{
  return rcutils_expand_user(path, rcutils_get_default_allocator());
}
}  // namespace ros2
