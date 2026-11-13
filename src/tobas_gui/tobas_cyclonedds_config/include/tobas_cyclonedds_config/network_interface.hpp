// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>
#include <string>

namespace tobas::cyclonedds
{
struct NetworkInterface
{
  std::string name;
  std::optional<int> priority;
  std::optional<bool> multicast;
};
}  // namespace tobas::cyclonedds
