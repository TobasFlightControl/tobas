// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_crypt/base.hpp"

#include <crypt.h>

#include <cstring>
#include <iostream>

#include <tobas_linux/error.hpp>

namespace tobas
{
namespace crypt
{
std::string Crypt::crypt(const std::string& password) const
{
  struct crypt_data data;
  std::memset(&data, 0, sizeof(data));

  const auto salt = createSalt();
  const auto out = ::crypt_r(password.c_str(), salt.c_str(), &data);
  if (!out) {
    std::cerr << "crypt_r failed: " << linux::strError() << std::endl;
    return {};
  }

  return std::string(out);
}
}  // namespace crypt
}  // namespace tobas
