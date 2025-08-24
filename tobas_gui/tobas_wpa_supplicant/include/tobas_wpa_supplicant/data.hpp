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
  CountryCode country;
  std::string ctrl_interface;
  bool update_config;
  std::vector<Network> networks;
};
}  // namespace wpa
}  // namespace tobas
