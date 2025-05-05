#include "../include/tobas_linux/core.hpp"

#include <unistd.h>

#include <stdexcept>

using namespace std;
namespace fs = filesystem;

namespace linux
{
string userName()
{
  if (isSuperUser()) {
    return "root";
  }
  else {
    const auto user_name = getenv("USER");
    if (!user_name) {
      throw runtime_error("USER environment variable not set.");
    }
    return string(user_name);
  }
}

fs::path homeDir()
{
  if (isSuperUser()) {
    return "/root";
  }
  else {
    const auto home_dir = getenv("HOME");
    if (!home_dir) {
      throw runtime_error("HOME environment variable not set.");
    }
    return home_dir;
  }
}

fs::path expandUser(const string& path)
{
  if (path.substr(0, 2) == "~/") {
    return homeDir() / path.substr(2);
  }
  else {
    return path;
  }
}

bool isSuperUser()
{
  return getuid() == 0;
}
}  // namespace linux
