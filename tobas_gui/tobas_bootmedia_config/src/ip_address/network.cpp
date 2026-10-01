// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/ip_address/network.hpp"

#include <charconv>
#include <string_view>

#include <arpa/inet.h>

#define SI_NO_CONVERSION
#include <SimpleIni.h>

namespace tobas
{
namespace gui
{
namespace bm
{
namespace
{
constexpr char kMatchSection[] = "Match";
constexpr char kNetworkSection[] = "Network";
constexpr char kNameKey[] = "Name";
constexpr char kDhcpKey[] = "DHCP";
constexpr char kAddressKey[] = "Address";
constexpr char kGatewayKey[] = "Gateway";
constexpr char kDnsKey[] = "DNS";

std::optional<uint32_t> ipv4StringToInt(const std::string& text)
{
  in_addr addr{};
  if (inet_pton(AF_INET, text.c_str(), &addr) != 1) {
    return std::nullopt;
  }
  return ntohl(addr.s_addr);
}

std::string ipv4IntToString(uint32_t _addr)
{
  in_addr addr{};
  addr.s_addr = htonl(_addr);  // little endian -> big endian

  char buffer[INET_ADDRSTRLEN]{};
  inet_ntop(AF_INET, &addr, buffer, sizeof(buffer));

  return std::string(buffer);
}

std::optional<uint8_t> prefixStringToInt(const std::string& text)
{
  uint32_t prefix{};
  const auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), prefix);
  if (ec != std::errc{} || ptr != text.data() + text.size() || prefix > 32) {
    return std::nullopt;
  }
  return static_cast<uint8_t>(prefix);
}

std::expected<Network::Manual, std::string> parseAddressLine(const std::string& text)
{
  const auto slash_pos = text.find('/');
  if (slash_pos == std::string::npos) {
    return std::unexpected("Missing '/' in address.");
  }

  const auto address = ipv4StringToInt(text.substr(0, slash_pos));
  if (!address) {
    return std::unexpected("Failed to get IPv4 address.");
  }

  const auto prefix = prefixStringToInt(text.substr(slash_pos + 1));
  if (!prefix) {
    return std::unexpected("Failed to get IPv4 prefix.");
  }
  return Network::Manual{ .address = *address, .prefix = *prefix, .gateway = {}, .dns = {} };
}

std::string prefixIntToString(uint8_t _prefix)
{
  return '/' + std::to_string(_prefix);
}
}  // namespace

std::expected<Network, std::string> loadNetwork(const std::string& path)
{
  Network network;
  CSimpleIniCaseA ini;
  ini.SetUnicode(true);
  ini.SetMultiKey(true);

  if (ini.LoadFile(path.c_str()) != SI_OK) {
    return std::unexpected("INI load failed.");
  }

  network.name = ini.GetValue(kMatchSection, kNameKey, "");
  if (network.name.empty()) {
    return std::unexpected("NIC name is not defined.");
  }

  network.automatic = ini.GetBoolValue(kNetworkSection, kDhcpKey, false);

  if (!network.automatic) {
    // Address + Prefix
    {
      const std::string address_text = ini.GetValue(kNetworkSection, kAddressKey, "");
      if (address_text.empty()) {
        network.automatic = true;
        network.manual = {};
        return network;
      }
      const auto address = parseAddressLine(address_text);
      if (!address) {
        return std::unexpected("Failed to parse address line '" + address_text + "': " + address.error());
      }
      network.manual = *address;
    }

    // Gateway
    {
      const std::string gateway_text = ini.GetValue(kNetworkSection, kGatewayKey, "");
      if (gateway_text.empty()) {
        network.automatic = true;
        network.manual = {};
        return network;
      }
      const auto gateway = ipv4StringToInt(gateway_text);
      if (!gateway) {
        return std::unexpected("Failed to parse gateway line '" + gateway_text + "'.");
      }
      network.manual.gateway = *gateway;
    }

    // DNS
    CSimpleIniCaseA::TNamesDepend dnss;
    if (ini.GetAllValues(kNetworkSection, kDnsKey, dnss)) {
      if (dnss.size() == 0) {
        network.automatic = true;
        network.manual = {};
        return network;
      }
      for (const auto& dns : dnss) {
        if (!dns.pItem) {
          return std::unexpected("DNS is null.");
        }
        const auto value = ipv4StringToInt(dns.pItem);
        if (!value) {
          return std::unexpected("Failed to parse DNS line '" + std::string(dns.pItem) + "'.");
        }
        network.manual.dns.push_back(*value);
      }
    }
  }

  return network;
}

bool saveNetwork(const std::string& path, const Network& network)
{
  CSimpleIniCaseA ini;
  ini.SetUnicode(true);
  ini.SetMultiKey(true);

  ini.SetValue(kMatchSection, kNameKey, network.name.c_str());

  if (network.automatic) {
    ini.SetValue(kNetworkSection, kDhcpKey, "yes");
  }
  else {
    const auto address_text = ipv4IntToString(network.manual.address) + prefixIntToString(network.manual.prefix);
    const auto gateway_text = ipv4IntToString(network.manual.gateway);
    ini.SetValue(kNetworkSection, kAddressKey, address_text.c_str());
    ini.SetValue(kNetworkSection, kGatewayKey, gateway_text.c_str());
    for (const auto& dns : network.manual.dns) {
      const auto dns_text = ipv4IntToString(dns);
      ini.SetValue(kNetworkSection, kDnsKey, dns_text.c_str());
    }
  }

  if (ini.SaveFile(path.c_str()) != SI_OK) {
    return false;
  }

  return true;
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
