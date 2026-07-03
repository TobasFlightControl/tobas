// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include "./data.hpp"

namespace tobas
{
namespace ssh
{
namespace ak
{
/* Convert an SSH public key to a display string. */
std::expected<std::string, std::string> prettify(const Data& src);
}  // namespace ak
}  // namespace ssh
}  // namespace tobas
