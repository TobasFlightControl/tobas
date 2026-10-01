// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include "./data.hpp"

namespace tobas
{
namespace wpa
{
/** Parse wpa_supplicant configuration text. */
std::expected<Data, std::string> parseFromText(const std::string& text);
}  // namespace wpa
}  // namespace tobas
