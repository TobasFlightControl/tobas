// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas
{
namespace crypt
{
/* Update the user password. */
bool setShadowPassword(
  const std::string& _shadow_path,
  const std::string& _username,
  const std::string& _new_password,
  const Crypt& _crypt);
}  // namespace crypt
}  // namespace tobas
