#include <iostream>

#include <tobas_git/core.hpp>

using namespace std;

int main()
{
  const auto name = git::getGitConfigValue("user.name");
  const auto email = git::getGitConfigValue("user.email");

  cout << "Git User Name: " << name << endl;
  cout << "Git Email: " << email << endl;
}
