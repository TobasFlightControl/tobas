// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas::crypt
{
class Yescrypt : public Crypt
{
public:
  explicit Yescrypt();

private:
  std::expected<std::string, std::string> createSalt() const override;
};
}  // namespace tobas::crypt
