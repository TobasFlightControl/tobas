// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./data.hpp"

namespace tobas
{
namespace cyclonedds
{
bool parseFromText(const std::string& text, Data& dst);
}  // namespace cyclonedds
}  // namespace tobas
