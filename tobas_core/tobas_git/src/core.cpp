#include <iostream>
#include <git2.h>

#include "../include/tobas_git/core.hpp"

namespace git
{
std::string getGitConfigValue(const char* key)
{
  git_libgit2_init();

  git_config* config = nullptr;
  git_config_entry* entry = nullptr;
  std::string value = "";

  if (git_config_open_default(&config) == 0)
  {
    if (git_config_get_entry(&entry, config, key) == 0)
    {
      value = entry->value;
      git_config_entry_free(entry);
    }
    git_config_free(config);
  }

  git_libgit2_shutdown();

  return value;
}
}  // namespace git
