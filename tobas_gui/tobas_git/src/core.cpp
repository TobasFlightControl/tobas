#include "tobas_git/core.hpp"

#include <iostream>

#include <git2.h>

namespace tobas
{
namespace git
{
std::string getGitConfigValue(const char* key)
{
  git_libgit2_init();

  git_config* config = nullptr;
  git_config_entry* entry = nullptr;
  std::string value = "";

  if (git_config_open_default(&config) == 0) {
    if (git_config_get_entry(&entry, config, key) == 0) {
      value = entry->value;  // git側のメモリは開放されるためコピーする必要がある
      git_config_entry_free(entry);
    }
    else {
      std::cerr << "Failed to get git config entry: \"" << key << "\"" << std::endl;
    }

    git_config_free(config);
  }
  else {
    std::cerr << "Failed to open git config." << std::endl;
  }

  git_libgit2_shutdown();

  return value;
}
}  // namespace git
}  // namespace tobas
