#include "tobas_git/core.hpp"

#include <iostream>

#include <git2.h>

using namespace std;

namespace git
{
string getGitConfigValue(const char* key)
{
  git_libgit2_init();

  git_config* config = nullptr;
  git_config_entry* entry = nullptr;
  string value = "";

  if (git_config_open_default(&config) == 0) {
    if (git_config_get_entry(&entry, config, key) == 0) {
      value = entry->value;
      git_config_entry_free(entry);
    }
    else {
      cerr << "Failed to get git config entry: \"" << key << "\"" << endl;
    }

    git_config_free(config);
  }
  else {
    cerr << "Failed to open git config." << endl;
  }

  git_libgit2_shutdown();

  return value;
}
}  // namespace git
