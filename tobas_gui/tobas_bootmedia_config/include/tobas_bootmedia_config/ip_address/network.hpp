// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstdint>
#include <expected>
#include <string>
#include <vector>

namespace tobas
{
namespace gui
{
namespace bm
{
struct Network
{
  std::string name;       ///< e.g. wlan0, eth0
  bool automatic = true;  ///< If true, DHCP is used.

  /** Fixed IP address configuration */
  struct Manual
  {
    uint32_t address = {};      ///< e.g. 192.168.3.5
    uint8_t prefix = {};        ///< e.g. /24
    uint32_t gateway = {};      ///< e.g. 192.168.3.1
    std::vector<uint32_t> dns;  ///< e.g. 192.168.3.1, 1.1.1.1
  } manual;
};

std::expected<Network, std::string> loadNetwork(const std::string& path);
bool saveNetwork(const std::string& path, const Network& network);
}  // namespace bm
}  // namespace gui
}  // namespace tobas
