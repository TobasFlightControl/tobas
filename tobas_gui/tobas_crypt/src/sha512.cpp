// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_crypt/sha512.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cinttypes>
#include <iostream>
#include <vector>

#include <tobas_linux/error.hpp>

namespace tobas
{
namespace crypt
{
namespace
{
constexpr char kUrandomPath[] = "/dev/urandom";
constexpr size_t kLength = 16;
}  // namespace

Sha512::Sha512(int rounds) : rounds_(rounds)
{
}

std::string Sha512::createSalt() const
{
  const auto fd = ::open(kUrandomPath, O_RDONLY);
  if (fd < 0) {
    std::cerr << "Failed to open " << kUrandomPath << ": " << linux::strError() << std::endl;
    return {};
  }

  std::vector<uint8_t> buf(kLength);
  const auto n = ::read(fd, buf.data(), buf.size());
  ::close(fd);

  if (n != static_cast<ssize_t>(buf.size())) {
    std::cerr << "Failed to read urandom." << std::endl;
    return {};
  }

  constexpr char tbl[] = "./0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  std::string salt;
  salt.resize(kLength);
  for (size_t i = 0; i < kLength; ++i) {
    salt[i] = tbl[buf[i] & 63];  // 0..63 -> 64 types.
  }

  return "$6$rounds=" + std::to_string(rounds_) + "$" + salt;
}
}  // namespace crypt
}  // namespace tobas
