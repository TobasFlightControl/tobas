#include <iostream>

#include <tobas_linux/core.hpp>

using namespace std;

int main()
{
  const char* cmd = "date";
  const auto res = linux::executeCommand(cmd);
  cout << "Command: " << cmd << endl;
  cout << "Result : " << res << endl;
}
