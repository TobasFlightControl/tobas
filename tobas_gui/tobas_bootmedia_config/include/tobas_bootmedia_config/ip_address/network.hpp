// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace tobas
{
namespace gui
{
namespace bm
{
class Network
{
  static constexpr char kMatchSection[] = "Match";
  static constexpr char kNetworkSection[] = "Network";

  static constexpr char kNameKey[] = "Name";
  static constexpr char kDhcpKey[] = "DHCP";
  static constexpr char kAddressKey[] = "Address";
  static constexpr char kGatewayKey[] = "Gateway";
  static constexpr char kDnsKey[] = "DNS";

public:
  explicit Network();

  void clear();

  bool load(const std::string& path);
  bool save(const std::string& path) const;

  std::string name;  // e.g. wlan0, eth0
  bool automatic;    // If true, DHCP is used.

  /* Fixed IP address configuration */
  struct Manual
  {
    uint32_t address;           // e.g. 192.168.3.5
    uint8_t prefix;             // e.g. /24
    uint32_t gateway;           // e.g. 192.168.3.1
    std::vector<uint32_t> dns;  // e.g. 192.168.3.1, 1.1.1.1
  } manual;

private:
  void setAutomatic();
  bool parseAddressLine(const std::string& text);
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
