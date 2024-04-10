#include <array>
#include <memory>
#include <unistd.h>

#include "../include/tobas_std_tools/unix.hpp"

using namespace std;

namespace tobas_std
{
bool isSuperUser()
{
  return getuid() == 0;
}

string exec_command(const char* cmd)
{
  array<char, 128> buffer;
  string result;
  unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe)
    throw runtime_error("popen() failed!");
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    result += buffer.data();
  return result;
}
}  // namespace tobas_std
