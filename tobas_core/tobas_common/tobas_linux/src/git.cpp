#include "tobas_linux/git.hpp"

#include <iostream>

using namespace std;

namespace linux
{
GitHandler::GitHandler()
{
}

string GitHandler::getUserName()
{
  if (!command_executor_.execute("git config --global user.name")) {
    cerr << "Failed to get Git user name." << endl;
    return "";
  }

  return command_executor_.getOutput();
}

string GitHandler::getUserEmail()
{
  if (!command_executor_.execute("git config --global user.email")) {
    cerr << "Failed to get Git user email." << endl;
    return "";
  }

  return command_executor_.getOutput();
}
}  // namespace linux
