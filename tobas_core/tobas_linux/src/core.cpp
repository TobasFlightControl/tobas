#include <array>
#include <memory>
#include <unistd.h>

#include "../include/tobas_linux/core.hpp"

using namespace std;

namespace linux
{
string userName()
{
  if (isSuperUser())
  {
    return "root";
  }
  else
  {
    const auto user_name = getenv("USER");
    if (user_name == nullptr)
      throw runtime_error("USER environment variable not set.");
    return string(user_name);
  }
}

string homeDir()
{
  if (isSuperUser())
  {
    return "/root";
  }
  else
  {
    const auto home_dir = getenv("HOME");
    if (home_dir == nullptr)
      throw runtime_error("HOME environment variable not set.");
    return string(home_dir);
  }
}

string expandUser(const string& path)
{
  if (path.size() > 0 && path[0] == '~')
    return homeDir() + path.substr(1);
  else
    return path;
}

bool isSuperUser()
{
  return getuid() == 0;
}

string executeCommand(const char* cmd)
{
  array<char, 128> buffer;
  string result;
  unique_ptr<FILE, int (*)(FILE*)> pipe(popen(cmd, "r"), pclose);
  if (!pipe)
    throw runtime_error("popen() failed.");
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    result += buffer.data();
  return result;
}

std::string getGitUserName()
{
  return executeCommand("git config --global user.name");
}

std::string getGitUserEmail()
{
  return executeCommand("git config --global user.email");
}
}  // namespace linux
