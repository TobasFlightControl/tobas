// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <vector>

#include "./network_interface.hpp"
#include "./shared_memory.hpp"

namespace tobas
{
namespace cyclonedds
{
struct Data
{
  std::vector<NetworkInterface> interfaces;
  bool redundant_networking = false;
  SharedMemory shared_memory;
};
}  // namespace cyclonedds
}  // namespace tobas
