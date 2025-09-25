#include <iostream>

#include <tobas_git/core.hpp>

int main()
{
  const auto name = git::getGitConfigValue("user.name");
  const auto email = git::getGitConfigValue("user.email");

  std::cout << "Git User Name: " << name << std::endl;
  std::cout << "Git Email: " << email << std::endl;
}
