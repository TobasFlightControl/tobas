#pragma once

#include <string>

#include "./key_mgmt.hpp"

namespace tobas
{
namespace wpa
{
struct Network
{
  std::string ssid = "";
  std::string psk = "";
  int priority = 0;
  bool scan_ssid = false;
  KeyMgmt key_mgmt = NONE;
};
}  // namespace wpa
}  // namespace tobas
