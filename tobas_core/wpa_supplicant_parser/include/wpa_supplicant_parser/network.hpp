#pragma once

#include <string>

#include "./key_mgmt.hpp"

namespace wpa
{
struct Network
{
  std::string ssid = "";
  std::string psk = "";
  key_mgmt_t key_mgmt = WPA_PSK;
  int priority = 0;
};
}  // namespace wpa
