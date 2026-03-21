#pragma once

#include <vector>

#include "./country_code.hpp"
#include "./network.hpp"

namespace tobas
{
namespace wpa
{
struct Data
{
  CountryCode country = JP;
  std::string ctrl_interface = "DIR=/var/run/wpa_supplicant";
  bool update_config = true;
  std::vector<Network> networks;
};
}  // namespace wpa
}  // namespace tobas
