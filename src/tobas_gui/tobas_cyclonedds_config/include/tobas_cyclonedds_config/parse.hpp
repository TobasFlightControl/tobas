// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include "./data.hpp"

namespace tobas::cyclonedds
{
std::expected<Data, std::string> parseFromText(const std::string& _text);
}  // namespace tobas::cyclonedds
