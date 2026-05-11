#include "tobas_bootmedia_config/ip_address/network.hpp"

#include <charconv>
#include <iostream>

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
bool ipv4StringToInt(const std::string& text, uint32_t& out)
{
  in_addr addr;
  if (inet_pton(AF_INET, text.c_str(), &addr) != 1) {
    std::cerr << "Failed to parse the IP address." << std::endl;
  }
  out = addr.s_addr;
  return true;
}

std::string ipv4IntToString(uint32_t _addr)
{
  in_addr addr{};
  addr.s_addr = _addr;

  char buffer[INET_ADDRSTRLEN]{};
  inet_ntop(AF_INET, &addr, buffer, sizeof(buffer));

  return std::string(buffer);
}

bool prefixStringToInt(const std::string& text, uint8_t& out)
{
  const auto begin = text.data();
  const auto end = text.data() + text.size();

  uint32_t prefix{};  // Avoid [maybe-uninitialized]
  const auto [ptr, ec] = std::from_chars(begin, end, prefix);

  if (ec != std::errc{} || ptr != end) {
    std::cerr << "Failed to parse the prefix length." << std::endl;
  }
  if (prefix > 32) {
    std::cerr << "Invalid prefix length: " << prefix << std::endl;
  }

  out = static_cast<uint8_t>(prefix);
  return true;
}

std::string prefixIntToString(uint8_t _prefix)
{
  return '/' + std::to_string(_prefix);
}
}  // namespace

Network::Network()
{
}

void Network::clear()
{
  name.clear();
  setAutomatic();
}

bool Network::load(const std::string& path)
{
  CSimpleIniCaseA ini;
  ini.SetUnicode(true);
  ini.SetMultiKey(true);

  if (ini.LoadFile(path.c_str()) != SI_OK) {
    std::cerr << "Failed to load " << path << "." << std::endl;
    return false;
  }

  clear();

  name = ini.GetValue(kMatchSection, kNameKey, "");
  if (name.empty()) {
    std::cerr << "NIF name is not defined." << std::endl;
    return false;
  }

  automatic = ini.GetBoolValue(kNetworkSection, kDhcpKey, false);

  if (!automatic) {
    // Address + Prefix
    {
      const std::string address_text = ini.GetValue(kNetworkSection, kAddressKey, "");
      if (address_text.empty()) {
        std::cout << kNetworkSection << "." << kAddressKey << " is not defined." << std::endl;
        setAutomatic();
        return true;
      }
      if (!parseAddressLine(address_text)) {
        std::cerr << "Failed to parse " << kNetworkSection << "." << kAddressKey << "." << std::endl;
        return false;
      }
    }

    // Gateway
    {
      const std::string gateway_text = ini.GetValue(kNetworkSection, kGatewayKey, "");
      if (gateway_text.empty()) {
        std::cout << kNetworkSection << "." << kGatewayKey << " is not defined." << std::endl;
        setAutomatic();
        return true;
      }
      if (!ipv4StringToInt(gateway_text, manual.gateway)) {
        std::cerr << "Failed to parse " << kNetworkSection << "." << kGatewayKey << "." << std::endl;
        return false;
      }
    }

    // DNS
    CSimpleIniCaseA::TNamesDepend dnss;
    if (ini.GetAllValues(kNetworkSection, kDnsKey, dnss)) {
      if (dnss.size() == 0) {
        std::cout << kNetworkSection << "." << kDnsKey << " is not defined." << std::endl;
        setAutomatic();
        return true;
      }
      for (const auto& dns : dnss) {
        if (!dns.pItem) {
          std::cerr << "DNS is null." << std::endl;
          return false;
        }
        manual.dns.emplace_back();
        if (!ipv4StringToInt(dns.pItem, manual.dns.back())) {
          std::cerr << "Failed to parse " << kNetworkSection << "." << kDnsKey << "." << std::endl;
          return false;
        }
      }
    }
  }

  return true;
}

bool Network::save(const std::string& path) const
{
  CSimpleIniCaseA ini;
  ini.SetUnicode(true);
  ini.SetMultiKey(true);

  ini.SetValue(kMatchSection, kNameKey, name.c_str());

  if (automatic) {
    ini.SetValue(kNetworkSection, kDhcpKey, "yes");
  }
  else {
    const auto address_text = ipv4IntToString(manual.address) + prefixIntToString(manual.prefix);
    const auto gateway_text = ipv4IntToString(manual.gateway);
    ini.SetValue(kNetworkSection, kAddressKey, address_text.c_str());
    ini.SetValue(kNetworkSection, kGatewayKey, gateway_text.c_str());
    for (const auto& dns : manual.dns) {
      const auto dns_text = ipv4IntToString(dns);
      ini.SetValue(kNetworkSection, kDnsKey, dns_text.c_str());
    }
  }

  if (ini.SaveFile(path.c_str()) != SI_OK) {
    std::cerr << "Failed to save " << path << "." << std::endl;
    return false;
  }

  return true;
}

void Network::setAutomatic()
{
  automatic = true;

  manual.address = 0;
  manual.prefix = 0;
  manual.gateway = 0;
  manual.dns.clear();
}

bool Network::parseAddressLine(const std::string& text)
{
  const auto slash_pos = text.find('/');
  if (slash_pos == std::string_view::npos) {
    std::cerr << "Failed to find \"/\" in address." << std::endl;
    return false;
  }

  const auto ip_part = text.substr(0, slash_pos);
  const auto prefix_part = text.substr(slash_pos + 1);

  if (ip_part.empty() || prefix_part.empty()) {
    std::cerr << "Failed to separate IP part and prefix part." << std::endl;
    return false;
  }

  if (!ipv4StringToInt(ip_part, manual.address)) {
    return false;
  }
  if (!prefixStringToInt(prefix_part, manual.prefix)) {
    return false;
  }

  return true;
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
