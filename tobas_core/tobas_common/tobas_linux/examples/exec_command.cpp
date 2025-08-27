#include <iostream>

#include <tobas_linux/command_executor.hpp>

using namespace std;

int main()
{
  linux::CommandExecutor command_executor;

  const char* cmd = "date";
  if (!command_executor.execute(cmd)) {
    cerr << "Command failed." << endl;
    return EXIT_FAILURE;
  }

  cout << "Command: " << cmd << endl;
  cout << "Result : " << command_executor.getOutput() << endl;

  return EXIT_SUCCESS;
}
