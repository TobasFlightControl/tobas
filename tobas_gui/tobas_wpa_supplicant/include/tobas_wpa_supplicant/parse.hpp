#pragma once

#include "./data.hpp"

namespace tobas
{
namespace wpa
{
class Parser
{
public:
  explicit Parser();

  bool parseFromText(const std::string& text, Data& dst);

private:
  static bool parseCountryCode(const std::string& src, CountryCode& dst);
  static bool parseKeyManagement(const std::string& src, KeyMgmt& dst);
};
}  // namespace wpa
}  // namespace tobas
