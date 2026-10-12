// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

namespace tobas
{
namespace crypt
{
class Crypt
{
public:
  std::expected<std::string, std::string> crypt(const std::string& password) const;

private:
  virtual std::expected<std::string, std::string> createSalt() const = 0;
};
}  // namespace crypt
}  // namespace tobas
