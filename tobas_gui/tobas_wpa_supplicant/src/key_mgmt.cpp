#include "tobas_wpa_supplicant/key_mgmt.hpp"

#include <iostream>

#define NONE_TOKEN "NONE"
#define WPA_PSK_TOKEN "WPA-PSK"
#define SAE_TOKEN "SAE"

#define NONE_LABEL "None"
#define WPA_PSK_LABEL "WPA2-Personal"
#define SAE_LABEL "WPA3-Personal"

namespace tobas
{
namespace wpa
{
std::string tokenFromEnum(KeyMgmt key_mgmt)
{
  switch (key_mgmt) {
    case KeyMgmt::NONE:
      return NONE_TOKEN;
    case KeyMgmt::WPA_PSK:
      return WPA_PSK_TOKEN;
    case KeyMgmt::SAE:
      return SAE_TOKEN;
    default:
      throw;
  }
}

bool enumFromToken(const std::string& token, KeyMgmt& dst)
{
  if (token == NONE_TOKEN) {
    dst = KeyMgmt::NONE;
    return true;
  }
  else if (token == WPA_PSK_TOKEN) {
    dst = KeyMgmt::WPA_PSK;
    return true;
  }
  else if (token == SAE_TOKEN) {
    dst = KeyMgmt::SAE;
    return true;
  }
  else {
    std::cerr << "Key management \"" << token << "\" is not supported." << std::endl;
    return false;
  }
}

std::string labelFromEnum(KeyMgmt key_mgmt)
{
  switch (key_mgmt) {
    case KeyMgmt::NONE:
      return NONE_LABEL;
    case KeyMgmt::WPA_PSK:
      return WPA_PSK_LABEL;
    case KeyMgmt::SAE:
      return SAE_LABEL;
    default:
      throw;
  }
}

bool enumFromLabel(const std::string& label, KeyMgmt& dst)
{
  if (label == NONE_LABEL) {
    dst = KeyMgmt::NONE;
    return true;
  }
  else if (label == WPA_PSK_LABEL) {
    dst = KeyMgmt::WPA_PSK;
    return true;
  }
  else if (label == SAE_LABEL) {
    dst = KeyMgmt::SAE;
    return true;
  }
  else {
    std::cerr << "Invalid key management: " << label << std::endl;
    return false;
  }
}
}  // namespace wpa
}  // namespace tobas
