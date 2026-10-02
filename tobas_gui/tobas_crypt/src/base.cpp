// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_crypt/base.hpp"

#include <crypt.h>

#include <cstring>

#include <tobas_linux/error.hpp>

namespace tobas
{
namespace crypt
{
std::expected<std::string, std::string> Crypt::crypt(const std::string& password) const
{
  struct crypt_data data;
  std::memset(&data, 0, sizeof(data));

  const auto salt = createSalt();
  if (!salt) {
    return salt;
  }

  const auto out = ::crypt_r(password.c_str(), salt->c_str(), &data);
  if (!out || out[0] == '*') {
    return std::unexpected("crypt_r failed: " + linux::strError());
  }

  return std::string(out);
}
}  // namespace crypt
}  // namespace tobas
