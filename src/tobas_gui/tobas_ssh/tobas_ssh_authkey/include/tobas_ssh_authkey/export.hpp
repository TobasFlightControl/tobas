// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include "./data.hpp"

namespace tobas::ssh::ak
{
std::expected<std::string, std::string> exportLine(const Data& _src);
}  // namespace tobas::ssh::ak
