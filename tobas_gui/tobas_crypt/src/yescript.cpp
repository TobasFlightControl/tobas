// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_crypt/yescrypt.hpp"

#include <crypt.h>

#include <tobas_std_tools/error.hpp>

namespace tobas
{
namespace crypt
{
Yescrypt::Yescrypt()
{
}

std::expected<std::string, std::string> Yescrypt::createSalt() const
{
  char salt[CRYPT_GENSALT_OUTPUT_SIZE]{};
  if (!crypt_gensalt_rn("$y$", 0, nullptr, 0, salt, sizeof(salt))) {
    return std::unexpected("crypt_gensalt_rn failed: " + st::strError());
  }
  return std::string(salt);
}
}  // namespace crypt
}  // namespace tobas
