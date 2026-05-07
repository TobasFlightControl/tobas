// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

#include <libssh/libssh.h>

namespace tobas
{
namespace ssh
{
namespace ak
{
struct Data
{
  ssh_keytypes_e key_type = SSH_KEYTYPE_UNKNOWN;
  ssh_key key = nullptr;
  std::string comment = "";
};
}  // namespace ak
}  // namespace ssh
}  // namespace tobas
