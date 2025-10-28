#pragma once

#include "./data.hpp"

namespace tobas
{
namespace wpa
{
class Exporter
{
public:
  explicit Exporter();

  std::string exportText(const Data& src) const;

private:
  static const char* countryCodeToString(CountryCode cc);
};
}  // namespace wpa
}  // namespace tobas
