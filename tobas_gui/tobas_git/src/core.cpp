// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_git/core.hpp"

#include <git2.h>

namespace tobas
{
namespace git
{
std::expected<std::string, std::string> getGitConfigValue(const char* key)
{
  if (git_libgit2_init() < 0) {
    return std::unexpected("Failed to initialize libgit2.");
  }

  git_config* config = nullptr;
  git_config_entry* entry = nullptr;
  std::expected<std::string, std::string> value;

  if (git_config_open_default(&config) == 0) {
    if (git_config_get_entry(&entry, config, key) == 0) {
      value = entry->value ? entry->value : "";  // Copy before releasing memory owned by git.
      git_config_entry_free(entry);
    }
    else {
      value = std::unexpected("Failed to get git config entry: '" + std::string(key) + "'.");
    }

    git_config_free(config);
  }
  else {
    value = std::unexpected("Failed to open git config.");
  }

  git_libgit2_shutdown();

  return value;
}
}  // namespace git
}  // namespace tobas
