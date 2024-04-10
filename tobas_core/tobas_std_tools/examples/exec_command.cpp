#include <iostream>

#include <tobas_std_tools/unix.hpp>

using namespace std;

int main()
{
  const char* cmd = "date";
  const auto res = tobas_std::exec_command(cmd);
  cout << "Command: " << cmd << endl;
  cout << "Result : " << res << endl;
}
